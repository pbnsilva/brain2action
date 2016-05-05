import numpy as np
from utils import Quaternion, slerp_no_invert
from utils import Timer


class Model:

    def __init__(self, fpath):
        self._scale_factor = 0.1715316801
        self.position = [0, 0, 0]
        self.color = [1, 1, 1]
        self.skeleton = Skeleton()
        self.vertices = None
        self.normals = None

        self.original_vertices = None
        self.original_normals = None

        self.timer = Timer()
        self.iter = 0

        self.uvs = []
        self.vertex_indices = []
        self.normal_indices = []
        self.normal_to_vertex = []
        self.uv_indices = []
        self.influences = []
        self.animations = {}
        self._load(fpath)
        self.vertex_array = np.ndarray(shape=(len(self.vertex_indices) * 3,), dtype=float)
        self.normal_array = np.ndarray(shape=(len(self.vertex_indices) * 3,), dtype=float)
        self.uv_array = np.ndarray(shape=(len(self.vertex_indices) * 2,), dtype=float)
        self.transform_vertices()

    def rotate_joint(self, joint_id, rotation):
        self.skeleton.joints[joint_id].relative_rotation = rotation
        for k in self.skeleton.joints:  # TODO only need to update parent and children
            self.skeleton.joints[k].update_absolutes()
        # self.transform_vertices()

    def transform_vertices(self):
        t = self.timer

        t.start('1')
        # find the joint transformations and rotations
        joints = self.skeleton.joints
        transformations = {}
        rotations = {}
        for k in joints:
            joint = joints[k]
            transformations[k] = np.dot(np.dot(np.array([[self._scale_factor, 0, 0, 0], [0, self._scale_factor, 0, 0], [0, 0, self._scale_factor, 0], [0, 0, 0, 1]]), joint.get_absolute_transformation()), joint.original_get_absolute_transformation_inverse())
            rotations[k] = np.dot(joint.absolute_rotation.as_matrix(), joint.original_absolute_rotation.as_matrix().transpose())
        t.stop('1')

        t.start('2')
        # transform the vertices
        vertices = self.vertices
        org_vertices = self.original_vertices
        influences = self.influences
        num_vertices = len(vertices) / 3
        for vi in xrange(num_vertices):
            joint_trans = transformations[influences[vi][0][0]] * influences[vi][0][1]
            for joint_id, weight in influences[vi][1:]:
                joint_trans += transformations[joint_id] * weight
            vertex = np.dot(joint_trans, np.array([org_vertices[vi * 3], org_vertices[vi * 3 + 1], org_vertices[vi * 3 + 2], 1]))
            vertices[vi * 3] = vertex[0]
            vertices[vi * 3 + 1] = vertex[1]
            vertices[vi * 3 + 2] = vertex[2]
        t.stop('2')

        t.start('3')
        # transform the normals
        normals = self.normals
        org_normals = self.original_normals
        normal_to_vertex = self.normal_to_vertex
        num_normals = len(normals) / 3
        for ni in xrange(num_normals):
            vi = normal_to_vertex[ni]
            joint_rot = rotations[influences[vi][0][0]] * influences[vi][0][1]
            for join_id, weight in influences[vi][1:]:
                joint_rot += rotations[joint_id] * weight
            normal = np.dot(joint_rot, np.array([org_normals[ni * 3], org_normals[ni * 3 + 1], org_normals[ni * 3 + 2], 1]))
            normals[ni * 3] = normal[0]
            normals[ni * 3 + 1] = normal[1]
            normals[ni * 3 + 2] = normal[2]
        t.stop('3')

        t.start('4')
        # fill in vertex arrays
        vertex_array = self.vertex_array
        normal_array = self.normal_array
        vertex_indices = self.vertex_indices
        normal_indices = self.normal_indices
        for index in xrange(len(vertex_indices)):
            vi = vertex_indices[index]
            ni = normal_indices[index]
            vertex_array[index * 3] = vertices[vi * 3]
            vertex_array[index * 3 + 1] = vertices[vi * 3 + 1]
            vertex_array[index * 3 + 2] = vertices[vi * 3 + 2]
            normal_array[index * 3] = normals[ni * 3]
            normal_array[index * 3 + 1] = normals[ni * 3 + 1]
            normal_array[index * 3 + 2] = normals[ni * 3 + 2]
        t.stop('4')

        # print '1 ', t.get('1')
        # print '2 ', t.get('2')
        # print '3 ', t.get('3')
        # print '4 ', t.get('4')
        # print
        self.iter += 1

    def add_animation(self, name, animation):
        self.animations[name] = {'animation': animation, 'current_frame': 0}

    def update_animation(self, name, do_transform_vertices=True):
        animation = self.animations[name]['animation']

        # if we've reached the end of the animation loop
        if self.animations[name]['current_frame'] == self.animations[name]['animation'].num_frames - 1:
            self.animations[name]['current_frame'] = 1
        else:
            self.animations[name]['current_frame'] += 1

        num_tracks = animation.num_tracks
        for i in xrange(num_tracks):
            track = animation.tracks[i]

            # if the current frame is a keyframe
            if animation.is_keyframe(i, self.animations[name]['current_frame']):
                self.skeleton.joints[track['joint_id']].relative_rotation = animation.get_keyframe(i, self.animations[name]['current_frame'])[1]
            else:   # interpolate
                next_keyframe = animation.get_next_keyframe(i, self.animations[name]['current_frame'])
                prev_keyframe = animation.get_previous_keyframe(i, self.animations[name]['current_frame'])

                keyframe_span = 0.0
                if next_keyframe[0] < prev_keyframe[0]:
                    keyframe_span = animation.num_frames - prev_keyframe[0] + next_keyframe[0]
                else:
                    keyframe_span = next_keyframe[0] - prev_keyframe[0]

                keyframe_off = 0.0
                if self.animations[name]['current_frame'] < prev_keyframe[0]:
                    keyframe_off = animation.num_frames - prev_keyframe[0] + self.animations[name]['current_frame']
                else:
                    keyframe_off = self.animations[name]['current_frame'] - prev_keyframe[0]

                rotation = slerp_no_invert(prev_keyframe[1], next_keyframe[1], keyframe_off / float(keyframe_span))
                self.skeleton.joints[track['joint_id']].relative_rotation = rotation

        for k in self.skeleton.joints:
            self.skeleton.joints[k].update_absolutes()

        if do_transform_vertices:
            self.transform_vertices()

    def _load(self, fpath):
        with open(fpath, 'r') as f:
            line = f.readline()
            while line:
                if line.startswith('joints'):
                    joint_count = int(line.split()[1])
                    for j in xrange(joint_count):
                        id, name = f.readline().split()
                        joint = Joint(int(id), name)
                        joint.parent = int(f.readline())
                        joint.orientation = Quaternion(*map(float, f.readline().split()))
                        joint.relative_translation = map(float, f.readline().split())
                        joint.relative_rotation = Quaternion(*map(float, f.readline().split()))
                        joint.original_relative_translation = joint.relative_translation
                        joint.original_relative_rotation = joint.relative_rotation
                        joint.skeleton = self.skeleton
                        joint.original_update_absolutes()
                        joint.update_absolutes()
                        self.skeleton.add_joint(joint)
                elif line.startswith('vertices'):
                    vertice_count = int(line.split()[1])
                    self.vertices = np.ndarray((vertice_count * 3,), dtype=float)
                    self.original_vertices = np.ndarray((vertice_count * 3,), dtype=float)
                    for vi in xrange(vertice_count):
                        points = map(float, f.readline().split())
                        for i in xrange(3):
                            self.vertices[vi * 3 + i] = points[i]
                    np.copyto(self.original_vertices, self.vertices)
                elif line.startswith('normals'):
                    normals_count = int(line.split()[1])
                    self.normals = np.ndarray((normals_count * 3,), dtype=float)
                    self.original_normals = np.ndarray((normals_count * 3,), dtype=float)
                    for ni in xrange(normals_count):
                        points = map(float, f.readline().split())
                        for i in xrange(3):
                            self.normals[ni * 3 + i] = points[i]
                    np.copyto(self.original_normals, self.normals)
                elif line.startswith('uvs'):
                    uvs_count = int(line.split()[1])
                    for uv in xrange(uvs_count):
                        self.uvs += map(float, f.readline().split())
                elif line.startswith('triangles'):
                    triangle_count = int(line.split()[1])
                    self.normal_to_vertex = [0] * (len(self.normals) / 3)
                    for t in xrange(triangle_count):
                        vals = map(int, f.readline().split())
                        self.vertex_indices += vals[:3]
                        self.normal_indices += vals[3:6]
                        self.normal_to_vertex[vals[3]] = vals[0]
                        self.normal_to_vertex[vals[4]] = vals[1]
                        self.normal_to_vertex[vals[5]] = vals[2]
                        self.uv_indices += vals[6:]
                elif line.startswith('weights'):
                    weight_count = int(line.split()[1])
                    self.influences = [[] for _ in xrange(len(self.vertices) / 3)]
                    for w in xrange(weight_count):
                        vals = f.readline().split()
                        self.influences[int(vals[0])].append((int(vals[1]), float(vals[3])))
                line = f.readline()


class Skeleton:

    def __init__(self):
        self.joints = {}

    def add_joint(self, joint):
        self.joints[joint.id] = joint


class Joint:

    def __init__(self, id, name):
        self.id = id
        self.name = name
        self.parent = None
        self.orientation = None
        self.relative_translation = None
        self.relative_rotation = None
        self.absolute_translation = None
        self.absolute_rotation = None
        self.skeleton = None

        self.original_relative_translation = None
        self.original_relative_rotation = None
        self.original_absolute_translation = None
        self.original_absolute_rotation = None

    def update_absolutes(self):
        if self.parent != -1:
            parent_joint = self.skeleton.joints[self.parent]
            self.absolute_rotation = parent_joint.absolute_rotation * self.orientation * self.relative_rotation
            self.absolute_translation = parent_joint.absolute_translation + self.rotate_vector(self.relative_translation, parent_joint.absolute_rotation)
        else:
            q = self.orientation * self.relative_rotation
            self.absolute_rotation = Quaternion(*q.values)
            self.absolute_translation = self.relative_translation

    def original_update_absolutes(self):
        if self.parent != -1:
            parent_joint = self.skeleton.joints[self.parent]
            self.original_absolute_rotation = parent_joint.original_absolute_rotation * self.orientation * self.original_relative_rotation
            self.original_absolute_translation = parent_joint.original_absolute_translation + self.rotate_vector(self.original_relative_translation, parent_joint.original_absolute_rotation)
        else:
            q = self.orientation * self.original_relative_rotation
            self.original_absolute_rotation = Quaternion(*q.values)
            self.original_absolute_translation = self.original_relative_translation

    def rotate_vector(self, vector, quaternion):
        q_vector = Quaternion()
        q_vector.values[0] = vector[0]
        q_vector.values[1] = vector[1]
        q_vector.values[2] = vector[2]
        q_vector.values[3] = 0

        # rotate the pure quaternion
        q_result = quaternion * q_vector * quaternion.get_conjugate()

        return np.array([q_result.values[0], q_result.values[1], q_result.values[2]])

    def get_absolute_transformation(self):
        rot = self.absolute_rotation.as_matrix()

        trans = np.identity(4)
        trans[0, 3] = self.absolute_translation[0]
        trans[1, 3] = self.absolute_translation[1]
        trans[2, 3] = self.absolute_translation[2]

        return np.dot(trans, rot)

    def get_absolute_transformation_inverse(self):
        rot = np.transpose(self.absolute_rotation.as_matrix())

        abst = np.array([self.absolute_translation[0], self.absolute_translation[1], self.absolute_translation[2], 1])
        temp = np.dot(rot, abst)
        trans = np.identity(4)
        trans[0, 3] = -temp[0, 0]
        trans[1, 3] = -temp[0, 1]
        trans[2, 3] = -temp[0, 2]

        return np.dot(trans, rot)

    def original_get_absolute_transformation(self):
        rot = self.original_absolute_rotation.as_matrix()

        trans = np.identity(4)
        trans[0, 3] = self.original_absolute_translation[0]
        trans[1, 3] = self.original_absolute_translation[1]
        trans[2, 3] = self.original_absolute_translation[2]

        return np.dot(trans, rot)

    def original_get_absolute_transformation_inverse(self):
        rot = self.original_absolute_rotation.as_matrix().transpose()

        abst = np.array([self.original_absolute_translation[0], self.original_absolute_translation[1], self.original_absolute_translation[2], 1])
        temp = np.dot(rot, abst)
        trans = np.identity(4)
        trans[0, 3] = -temp[0]
        trans[1, 3] = -temp[1]
        trans[2, 3] = -temp[2]

        return np.dot(trans, rot)
