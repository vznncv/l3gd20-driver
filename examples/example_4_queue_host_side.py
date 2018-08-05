#!/usr/bin/env python3
import itertools
import sys
from collections import namedtuple

import importlib
import logging
import re
from contextlib import suppress
from threading import Thread

logger = logging.getLogger(__name__)

# check dependencies
DEPENDENCIES = {
    'numpy': 'numpy',
    'glumpy': 'glumpy',
    'click': 'click',
    'pySerial': 'serial'
}
_missed_libs = []
for lib_name, mod_name in DEPENDENCIES.items():
    try:
        importlib.import_module(mod_name)
    except ImportError:
        _missed_libs.append(lib_name)
if _missed_libs:
    print("Error. The following libraries are absent and should be installed.", file=sys.stderr)
    for lib_name in _missed_libs:
        print("  - {}.".format(lib_name), file=sys.stderr)
    sys.exit(1)

import click
import numpy as np
import serial.tools.list_ports
from numpy import linalg as la
from glumpy import app, gloo, gl, glm
from serial import Serial


class Model:
    """
    Helper object that holds model to draw.

    It contains:
      - model vertex coordinates
      - surface normals
      - indexes to draw triangles
      - textures
      - information about model material
    """
    _V_TYPE = [('position', np.float32, 3),
               ('texcoord', np.float32, 2),
               ('normal', np.float32, 3)]

    _MaterialParams = namedtuple('MaterialParams', ['ambient_k', 'diffusion_k', 'specular_k'])

    @staticmethod
    def _get_fake_texture(color=(0.5, 0.5, 0.5)):
        color = (np.array(color) * 255).astype(np.uint8)
        return np.tile(color, (16, 16, 1))

    def __init__(self, vertices, indices, texture=None, ambient_k=0.4, diffusion_k=0.8, specular_k=0.6):
        """

        :param vertices:
        :param indices:
        :param texture: texture
        :param ambient_k: material ambient coefficient
        :param diffusion_k: material diffusion coefficient
        :param specular_k: material specular coefficient
        """
        self._material_params = self._MaterialParams(ambient_k, diffusion_k, specular_k)

        if texture is None:
            texture = self._get_fake_texture()
        self._texture = texture

        if indices is not None:
            self._indices = np.asanyarray(indices, dtype=np.uint32)
            if self._indices.ndim != 1:
                raise ValueError("indices must be one dimensional array")
            if len(self._indices) % 3 != 0:
                raise ValueError("Number of indices doesn't have multiplicity of 3")
            indices = self._indices
        else:
            self._indices = None

        # check vertices
        v_dtype = vertices.dtype
        if v_dtype.names is None:
            raise ValueError("vertices should be Structured numpy array")
        if vertices.ndim != 1:
            raise ValueError("vertices must be one dimensional array")
        if 'position' not in v_dtype.names:
            raise ValueError("vertices should has 'position'")
        # copy vertices
        v = np.empty(vertices.size, dtype=self._V_TYPE)
        v['position'] = vertices['position']

        if 'texcoord' in v_dtype.names:
            v['texcoord'] = vertices['texcoord']
        else:
            v['texcoord'] = 0

        if 'normal' in v_dtype.names:
            v['normal'] = vertices['normal']
        else:
            # calculate normals
            position = vertices['position']
            indices = indices or np.arange(len(vertices))
            vec_1 = position[indices[1::3]] - position[indices[0::3]]
            vec_2 = position[indices[2::3]] - position[indices[0::3]]
            normal = np.cross(vec_1, vec_2, axis=-1)
            normal /= la.norm(normal, axis=-1)[..., np.newaxis]
            normal = np.repeat(normal, 3, axis=0)
            normal_view = v['normal']
            normal_view[...] = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            for i, n in zip(indices, normal):
                normal_view[i] += n
            vec_lengths = la.norm(normal_view, axis=-1)
            vec_lengths[vec_lengths == 0] = 1
            normal_view /= vec_lengths[..., np.newaxis]

        self._vertices = v

        # transformation matrices that should be applied to object to get vertexes global coordinates
        self.matrices = []

    @property
    def vertices(self):
        return self._vertices

    @property
    def indexes(self):
        return self._indices

    @property
    def texture(self):
        return self._texture

    @property
    def material(self):
        """
        Material parameters:

        - ambient coefficient
        - diffusion coefficient
        - specular coefficient
        """
        return self._material_params

    def get_model_matrix(self):
        """
        Get model transform matrix to convert local model coordinates to global
        :return:
        """
        m = np.eye(4, dtype=np.float32)
        for tr_m in reversed(self.matrices):
            m = tr_m @ m
        return m


class SimpleScene(object):
    """
    Helper scene object to draw models using simple Phong shading.

    The scene control camera position and has one source of light.
    """
    _VERTEX_SHADER = r'''
    uniform mat4 u_model; // model transform matrix
    uniform mat4 u_view; // camera coordinate transform matrix
    uniform mat4 u_projection; // camera projection matrix

    attribute vec3 a_position;
    attribute vec3 a_normal;
    attribute vec2 a_texcoord;

    varying vec3 v_normal_g; // normal n the global coordinates
    varying vec3 v_position_g; // vertex position in the global coordinates
    varying vec2 v_texcoord;

    void main() {
        vec4 position_g = u_model * vec4(a_position, 1.0);
        gl_Position = u_projection * u_view * position_g;

        v_position_g = position_g.xyz / position_g.w;
        // note use 3x3 matrix, as normals don't require translation
        v_normal_g  = mat3(u_model) * a_normal;
        v_texcoord = a_texcoord;
    }
    '''

    _FRAGMENT_SHADER = r'''
    uniform vec3 u_light_pos; // light source position in the global coordinates
    uniform vec3 u_view_pos; // camera position in the global coordinates
    uniform sampler2D u_texture;  // texture

    // parameters of the phong reflection model
    uniform float u_ambient_k;
    uniform float u_diffusion_k;
    uniform float u_specular_k;

    varying vec3 v_normal_g;
    varying vec3 v_position_g;
    varying vec2 v_texcoord;

    void main() {
        vec3 t_color = vec3(texture2D(u_texture, v_texcoord));

        // light model parameters
        vec3 light_src_dir = normalize(u_light_pos - v_position_g);
        vec3 camera_src_dir = normalize(u_view_pos - v_position_g);
        vec3 normal_dir = -normalize(v_normal_g); // TODO: check normal direction
        vec3 reflect_light_dir = reflect(-light_src_dir, normal_dir);

        float ambient_k = u_ambient_k;
        float diffusion_k = max(0, dot(light_src_dir, normal_dir)) * u_diffusion_k;
        float specular_k = max(0, pow(dot(camera_src_dir, reflect_light_dir), 16)) * u_specular_k;

        float light_k = diffusion_k + ambient_k + specular_k;

        gl_FragColor = vec4(t_color * light_k, 1.0);
    }
    '''
    _ModelInfo = namedtuple('ModelInfo', ['model', 'program'])

    def __init__(self):
        self._model_infos = []

        self._camera_projection = None
        self.set_camera_projection(45, 1.0, 2.0, 100.0)

        self._camera_view = None
        self._camera_pos = None
        self.set_camera_position(0, 0, -10)

        self._light_pos = None
        self.set_light_pos(0, 0, -20)

    def set_camera_position(self, x, y, z):
        self._camera_view = glm.translation(x, y, z)
        self._camera_pos = x, y, z

        for model_info in self._model_infos:
            program = model_info.program
            program['u_view'] = self._camera_view
            program['u_view_pos'] = self._camera_pos

    def set_camera_projection(self, fovy, aspect, znear, zfar):
        self._camera_projection = glm.perspective(fovy, aspect, znear, zfar)
        for model_info in self._model_infos:
            program = model_info.program
            program['u_projection'] = self._camera_projection

    def set_light_pos(self, x, y, z):
        self._light_pos = x, y, z
        for model_info in self._model_infos:
            program = model_info.program
            program['u_light_pos'] = self._light_pos

    def register_model(self, model: Model):
        program = gloo.Program(vertex=self._VERTEX_SHADER, fragment=self._FRAGMENT_SHADER)

        # vertex information
        program['a_position'] = model.vertices['position']
        program['a_normal'] = model.vertices['normal']
        program['a_texcoord'] = model.vertices['texcoord']

        # material properties
        program['u_ambient_k'] = model.material.ambient_k
        program['u_diffusion_k'] = model.material.diffusion_k
        program['u_specular_k'] = model.material.specular_k

        #
        program['u_texture'] = model.texture

        program['u_model'] = model.get_model_matrix()
        program['u_projection'] = self._camera_projection
        program['u_view'] = self._camera_view
        program['u_view_pos'] = self._camera_pos
        program['u_light_pos'] = self._light_pos

        self._model_infos.append(self._ModelInfo(model=model, program=program))

    def draw(self):
        for model, program in self._model_infos:
            # update model position
            program['u_model'] = model.get_model_matrix()
            # draw model
            program.draw(gl.GL_TRIANGLES, model.indexes)


def build_cube():
    position = np.array(list(itertools.product(*[[+1, -1]] * 3)), dtype=np.float32)
    # build texture
    colors = [
        [0x1F, 0x77, 0xB4],
        [0xFF, 0x7F, 0x0E],
        [0x2C, 0xA0, 0x2C],
        [0xD6, 0x27, 0x28],
        [0x94, 0x67, 0xBD],
        [0x8C, 0x56, 0x4B],
    ]
    texture = np.array([colors], dtype=np.uint8)
    color_texcoords = np.zeros((len(colors), 2), dtype=np.float32)
    color_texcoords[:, 1] = 0.5
    color_texcoords[:, 0] = np.linspace(0, 1, len(colors), endpoint=False) + 1.0 / (2 * len(colors))

    position_indices = np.array([
        # right side
        0, 2, 1,
        1, 2, 3,

        # face side
        1, 3, 5,
        3, 7, 5,

        # left side
        5, 4, 7,
        4, 6, 7,

        # back side
        0, 4, 6,
        0, 6, 2,

        # top side
        1, 4, 0,
        1, 5, 4,

        # bottom side
        3, 6, 7,
        3, 2, 6
    ])
    color_indices = np.array([
        # right side
        0, 0, 0,
        0, 0, 0,

        # face side
        1, 1, 1,
        1, 1, 1,

        # left side
        2, 2, 2,
        2, 2, 2,

        # back side
        3, 3, 3,
        3, 3, 3,

        # top side
        4, 4, 4,
        4, 4, 4,

        # bottom side
        5, 5, 5,
        5, 5, 5
    ])
    # normals
    normal = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [-1, 0, 0],
        [0, 0, 1],
        [0, 1, 0],
        [0, -1, 0],
    ], dtype=np.float32)

    vertices = np.empty((12 * 3,), dtype=[('position', np.float32, 3),
                                          ('texcoord', np.float32, 2)])
    vertices['position'] = position[position_indices]
    vertices['texcoord'] = color_texcoords[color_indices]

    return Model(
        vertices=vertices,
        indices=None,
        texture=texture,
        specular_k=2.0,
        diffusion_k=0.6,
        ambient_k=0.4

    )


class DemoCube:
    """
    Helper class that create window and cube to show gyroscope position.
    """

    _RAD_TO_DEG_K = 180 / np.pi

    def __init__(self):
        self._model = build_cube()
        self._rotation_matrix = np.eye(4, dtype=np.float32)
        self._model.matrices[:] = [self._rotation_matrix]
        self._scene = SimpleScene()
        self._scene.set_camera_position(0, 0, -10)
        self._scene.register_model(self._model)
        self._thread = None
        self._active = False

        self.x = 1.0
        self.y = 0.0
        self.z = 0.0
        self.angle = 0.0

    def start_async(self):
        if self._thread is not None:
            raise ValueError("The demo has been run.")

        window = app.Window()

        @window.event
        def on_init():
            # FIXME: it doesn't work here, so it' has been duplicated in the main cycle
            gl.glEnable(gl.GL_DEPTH_TEST)

        @window.event
        def on_resize(width, height):
            ratio = width / float(height)
            self._scene.set_camera_projection(45.0, ratio, 2.0, 100.0)

        @window.event
        def on_draw(dt):
            window.clear()
            if dt == 0:
                return
            # ensure that depth test is enabled
            gl.glEnable(gl.GL_DEPTH_TEST)
            # rotate cube
            try:
                rotation_matrix = np.eye(4, dtype=np.float32)
                glm.rotate(
                    rotation_matrix,
                    angle=self.angle * self._RAD_TO_DEG_K,
                    x=self.x,
                    y=self.y,
                    z=self.z
                )
                self._rotation_matrix[...] = rotation_matrix
            except Exception as e:
                logger.error(e)
            # draw scene
            self._scene.draw()

        @window.event
        def on_close():
            print("Window has been closed", file=sys.stderr)
            sys.exit(1)

        def run():
            backend = app.__backend__
            clock = app.__init__(backend=backend)
            count = len(backend.windows())
            while count:
                count = backend.process(clock.tick())
                if not self._active:
                    break
            with suppress(Exception):
                window.close()

        self._active = True
        self._thread = Thread(target=run)
        self._thread.start()

    def stop(self):
        if self._thread is None:
            raise ValueError("The demo hasn't been run")
        self._active = False
        self._thread.join()
        self._thread = None

    def __enter__(self):
        self.start_async()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


def select_serial_port_manually():
    com_port_infos = sorted(serial.tools.list_ports.comports(), key=lambda v: v.device)
    if not com_port_infos:
        click.secho("No comports have been detected", fg='red')
        sys.exit(1)
    elif len(com_port_infos) == 1:
        print("Found one comport: {}".format(com_port_infos[0]))
        if (not click.confirm('Should it be used?')):
            sys.exit(1)
        com_port_info = com_port_infos[0]
    else:
        click.echo("The following ports are found:")
        for i, com_port_info in enumerate(com_port_infos, start=1):
            click.echo("  {}. {}".format(i, com_port_info))
        click.echo("")
        i = click.prompt("Select port to use", type=click.IntRange(1, len(com_port_infos))) - 1
        com_port_info = com_port_infos[i]
    return com_port_info.device


_LINE_REGEXES = {
    'angle': re.compile(r'angle:?\s*(?P<angle>[+-]?\d+(?:\.\d*))'),
    'x': re.compile(r'x:?\s*(?P<x>[+-]?\d+(?:\.\d*))'),
    'y': re.compile(r'y:?\s*(?P<y>[+-]?\d+(?:\.\d*))'),
    'z': re.compile(r'z:?\s*(?P<z>[+-]?\d+(?:\.\d*))')
}


@click.command(context_settings=dict(help_option_names=['-h', '--help']))
@click.option('--port', help='Serial port', default=select_serial_port_manually)
@click.option('--baudrate', help='Serial port baudrate', default='115200', type=click.Choice(map(str, Serial.BAUDRATES)))
def main(port, baudrate):
    baudrate = int(baudrate)
    click.echo('Selected serial port:')
    click.echo('  - port {}'.format(port))
    click.echo('  - baudrate {}'.format(port))
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    with Serial(port=port, baudrate=baudrate) as ser, DemoCube() as demo_cube:
        while True:
            raw_line = ser.readline()
            try:
                line = raw_line.decode('utf-8').rstrip('\n')
            except Exception:
                logger.error("Invalid raw line: {}".format(raw_line))
                continue
            # parse line
            res = {}
            parsed = True
            for var_name, var_re in _LINE_REGEXES.items():
                m = var_re.search(line)
                if m is None:
                    parsed = False
                else:
                    res[var_name] = float(m.group(var_name))
            if not parsed:
                logger.error('Fail to parse line: {}'.format(line))
                continue

            # update cube and log line
            click.secho(line, fg='green')
            demo_cube.angle = res['angle']
            # Note display and stm32f3 axis order and direction differ
            demo_cube.x = res['x']
            demo_cube.y = res['z']
            demo_cube.z = -res['y']


if __name__ == '__main__':
    main()
