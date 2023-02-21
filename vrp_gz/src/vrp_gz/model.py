# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sdformat13 as sdf

from ament_index_python.packages import get_package_share_directory


import vrp_gz.bridges
import vrp_gz.payload_bridges

UAVS = []

USVS = [
    'booblik',
]

WAVEFIELD_SIZE = {'sydney_regatta': 1000,}


class Model:

    def __init__(self, model_name, model_type, position):
        self.model_name = model_name
        self.model_type = model_type
        self.position = position
        self.battery_capacity = 0
        self.wavefield_size = 0
        self.payload = {}

    def is_UAV(self):
        return self.model_type in UAVS

    def is_USV(self):
        return self.model_type in USVS

    def bridges(self):
        custom_launches = []
        nodes = []
        bridges = [
            # pose
            vrp_gz.bridges.pose(self.model_name),
            # pose static
            vrp_gz.bridges.pose_static(self.model_name),
        ]
        if self.is_UAV():
            pass
        elif self.is_USV():
            bridges.extend([
                # thrust cmd
                vrp_gz.bridges.thrust(self.model_name, 'left'),
                vrp_gz.bridges.thrust(self.model_name, 'right'),
                vrp_gz.bridges.thrust(self.model_name, 'back'),
            ])

        return [bridges, nodes, custom_launches]

    def payload_bridges(self, world_name, payloads=None):
        # payloads on usv and uav
        if not payloads:
            payloads = self.payload
        bridges, nodes, payload_launches = self.payload_bridges_impl(world_name, payloads)

        return [bridges, nodes, payload_launches]

    def payload_bridges_impl(self, world_name, payloads):
        bridges = []
        nodes = []
        payload_launches = []
        for sensor_name, value in payloads.items():
            link_name = value[0]
            sensor_type = value[1]

            bridges.extend(
                vrp_gz.payload_bridges.payload_bridges(
                    world_name, self.model_name, link_name, sensor_name, sensor_type))

        return [bridges, nodes, payload_launches]


    def set_payload(self, payload):
        # UAV specific
        self.payload = payload

    def generate(self):
        # TODO: здесь sdf
        model_sdf = 'sdf'

        # parse sdf for payloads if model is urdf
        if self.urdf != '':
            self.payload = self.payload_from_sdf(model_sdf)

        # for debugging generated sdf file
        # with open('/tmp/wamv.sdf', 'w') as f:
        #     f.write(model_sdf)
        # print(command)

        return model_sdf

    def payload_from_sdf(self, model_sdf):
        payload = {}
        root = sdf.Root()
        root.load_sdf_string(model_sdf)
        model = root.model()
        for link_index in range(model.link_count()):
            link = model.link_by_index(link_index)
            for sensor_index in range(link.sensor_count()):
                sensor = link.sensor_by_index(sensor_index)
                payload[sensor.name()] = [link.name(), sensor.type()]
        plugins = model.plugins()
        for plugin in plugins:
            payload[plugin.name()] = ['', plugin.filename()]
        return payload


    def spawn_args(self, model_sdf=None):
        if not model_sdf:
            model_sdf = self.generate()

        return ['-string', model_sdf,
                '-name', self.model_name,
                '-allow_renaming', 'false',
                '-x', str(self.position[0]),
                '-y', str(self.position[1]),
                '-z', str(self.position[2]),
                '-R', str(self.position[3]),
                '-P', str(self.position[4]),
                '-Y', str(self.position[5])]


