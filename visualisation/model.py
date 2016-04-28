import numpy as np
from utils import Quaternion, slerp_no_invert, Timer


class Model:

    def __init__(self, fpath):
        self._scale_factor = 0.1715316801
        self.skeleton = Skeleton()
        self.vertices = []
        self.normals = []

        self.original_vertices = []
        self.original_normals = []

        self.uvs = []
        self.vertex_indices = []
        self.normal_indices = []
        self.normal_to_vertex = []
        self.uv_indices = []
        self.influences = []
        self._animations = {}
        self._load(fpath)
        self.vertex_array = np.ndarray(shape=(len(self.vertex_indices) * 3,), dtype=float)
        self.normal_array = np.ndarray(shape=(len(self.vertex_indices) * 3,), dtype=float)
        self.uv_array = np.ndarray(shape=(len(self.vertex_indices) * 2,), dtype=float)
        self.timer = Timer()
        self.iter = 0
        self.transform_vertices()

    def transform_vertices(self):
        t = self.timer
        # find the joint transformations and rotations
        t.start('1')
        joints = self.skeleton.joints
        transformations = {}
        rotations = {}
        for k in joints:
            joint = joints[k]
            if k not in transformations:
                transformations[k] = np.ndarray(shape=(4, 4), dtype=float)
            if k not in rotations:
                rotations[k] = np.ndarray(shape=(4, 4), dtype=float)
            transformations[k] = np.array([[self._scale_factor, 0, 0, 0], [0, self._scale_factor, 0, 0], [0, 0, self._scale_factor, 0], [0, 0, 0, 1]]) * joint.get_absolute_transformation() * joint.original_get_absolute_transformation_inverse()
            rotations[k] = joint.absolute_rotation.as_matrix() * joint.original_absolute_rotation.as_matrix().transpose()
        t.stop('1')

        t.start('2')
        # transform the vertices
        org_vertices = self.original_vertices
        influences = self.influences
        for vi in xrange(len(influences)):
            t.start('2.1')
            joint_trans = np.sum(transformations[joint_id] * weight for joint_id, weight in influences[vi])
            t.stop('2.1')
            t.start('2.2')
            vertex = np.dot(joint_trans, np.array([org_vertices[vi * 3], org_vertices[vi * 3 + 1], org_vertices[vi * 3 + 2], 1]))
            t.stop('2.2')
            t.start('2.3')
            self.vertices[vi * 3] = vertex[0, 0]
            self.vertices[vi * 3 + 1] = vertex[0, 1]
            self.vertices[vi * 3 + 2] = vertex[0, 2]
            t.stop('2.3')
        t.stop('2')

        t.start('3')
        # transform the normals
        org_normals = self.original_normals
        normal_to_vertex = self.normal_to_vertex
        num_normals = len(org_normals) / 3
        for normal_index in xrange(num_normals):
            joint_rot = np.sum(rotations[joint_id] * weight for joint_id, weight in influences[normal_to_vertex[normal_index]])
            normal = np.dot(joint_rot, np.array([org_normals[normal_index * 3], org_normals[normal_index * 3 + 1], org_normals[normal_index * 3 + 2], 1]))
            self.normals[normal_index * 3] = normal[0, 0]
            self.normals[normal_index * 3 + 1] = normal[0, 1]
            self.normals[normal_index * 3 + 2] = normal[0, 2]
        t.stop('3')

        t.start('4')
        # fill in vertex arrays
        for index in xrange(len(self.vertex_indices)):
            self.vertex_array[index * 3] = self.vertices[self.vertex_indices[index] * 3]
            self.vertex_array[index * 3 + 1] = self.vertices[self.vertex_indices[index] * 3 + 1]
            self.vertex_array[index * 3 + 2] = self.vertices[self.vertex_indices[index] * 3 + 2]

            self.normal_array[index * 3] = self.normals[self.normal_indices[index] * 3]
            self.normal_array[index * 3 + 1] = self.normals[self.normal_indices[index] * 3 + 1]
            self.normal_array[index * 3 + 2] = self.normals[self.normal_indices[index] * 3 + 2]
        t.stop('4')

        # print self.iter
        # print '1 ', t.get('1')
        # print '2 ', t.get('2')
        # print '2.1 ', t.get('2.1')
        # print '2.2 ', t.get('2.2')
        # print '2.3 ', t.get('2.3')
        # print '3 ', t.get('3')
        # print '4 ', t.get('4')
        # print
        self.iter += 1

    def add_animation(self, name, animation):
        self._animations[name] = {'animation': animation, 'current_frame': 0}

    def update_animation(self, name):
        animation = self._animations[name]['animation']

        # if we've reached the end of the animation loop
        if self._animations[name]['current_frame'] == self._animations[name]['animation'].num_frames - 1:
            self._animations[name]['current_frame'] = 1
        else:
            self._animations[name]['current_frame'] += 1

        num_tracks = animation.num_tracks
        for i in xrange(num_tracks):
            track = animation.tracks[i]

            # if the current frame is a keyframe
            if animation.is_keyframe(i, self._animations[name]['current_frame']):
                self.skeleton.joints[track['joint_id']].relative_rotation = animation.get_keyframe(i, self._animations[name]['current_frame'])[1]
            else:   # interpolate
                next_keyframe = animation.get_next_keyframe(i, self._animations[name]['current_frame'])
                prev_keyframe = animation.get_previous_keyframe(i, self._animations[name]['current_frame'])

                keyframe_span = 0.0
                if next_keyframe[0] < prev_keyframe[0]:
                    keyframe_span = animation.num_frames - prev_keyframe[0] + next_keyframe[0]
                else:
                    keyframe_span = next_keyframe[0] - prev_keyframe[0]

                keyframe_off = 0.0
                if self._animations[name]['current_frame'] < prev_keyframe[0]:
                    keyframe_off = animation.num_frames - prev_keyframe[0] + self._animations[name]['current_frame']
                else:
                    keyframe_off = self._animations[name]['current_frame'] - prev_keyframe[0]

                rotation = slerp_no_invert(prev_keyframe[1], next_keyframe[1], keyframe_off / float(keyframe_span))
                self.skeleton.joints[track['joint_id']].relative_rotation = rotation

        for k in self.skeleton.joints:
            self.skeleton.joints[k].update_absolutes()

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
                    for v in xrange(vertice_count):
                        self.vertices += map(float, f.readline().split())
                    self.original_vertices = [v for v in self.vertices]
                elif line.startswith('normals'):
                    normals_count = int(line.split()[1])
                    for n in xrange(normals_count):
                        self.normals += map(float, f.readline().split())
                    self.original_normals = [v for v in self.normals]
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

        return np.mat(trans * rot)

    def get_absolute_transformation_inverse(self):
        rot = np.transpose(self.absolute_rotation.as_matrix())

        abst = np.array([self.absolute_translation[0], self.absolute_translation[1], self.absolute_translation[2], 1])
        temp = np.dot(rot, abst)
        trans = np.identity(4)
        trans[0, 3] = -temp[0, 0]
        trans[1, 3] = -temp[0, 1]
        trans[2, 3] = -temp[0, 2]

        return np.mat(trans * rot)

    def original_get_absolute_transformation(self):
        rot = self.original_absolute_rotation.as_matrix()

        trans = np.identity(4)
        trans[0, 3] = self.original_absolute_translation[0]
        trans[1, 3] = self.original_absolute_translation[1]
        trans[2, 3] = self.original_absolute_translation[2]

        return np.mat(trans * rot)

    def original_get_absolute_transformation_inverse(self):
        rot = self.original_absolute_rotation.as_matrix().transpose()

        abst = np.array([self.original_absolute_translation[0], self.original_absolute_translation[1], self.original_absolute_translation[2], 1])
        temp = np.dot(rot, abst)
        trans = np.identity(4)
        trans[0, 3] = -temp[0, 0]
        trans[1, 3] = -temp[0, 1]
        trans[2, 3] = -temp[0, 2]

        return np.mat(trans * rot)


class Animation:

    def __init__(self, fpath):
        self.num_frames = 0
        self.num_tracks = 0
        self.tracks = []
        self._load(fpath)

    def _load(self, fpath):
        with open(fpath, 'r') as f:
            self.num_frames, self.num_tracks = map(int, f.readline().split()[1:])
            for track_ind in xrange(self.num_tracks):
                joint_id = int(f.readline().split()[2])
                num_keyframes = int(f.readline().split()[1])
                track = {'joint_id': joint_id, 'frames': []}
                for keyframe_ind in xrange(num_keyframes):
                    vals = f.readline().split()
                    track['frames'].append((int(vals[0]), Quaternion(*map(float, vals[1:]))))   # (frame, rotation)
                self.tracks.append(track)

    def is_keyframe(self, track, frame):
        track = self.tracks[track]
        for i in xrange(len(track['frames'])):
            if frame == track['frames'][i][0]:
                return True
        return False

    def get_keyframe(self, track, frame):
        track = self.tracks[track]
        for i in xrange(len(track['frames'])):
            if frame == track['frames'][i][0]:
                return track['frames'][i]

    def get_next_keyframe(self, track, frame):
        track = self.tracks[track]
        keyframe = track['frames'][0]
        for i in xrange(len(track['frames'])):
            if frame < track['frames'][i][0]:
                keyframe = track['frames'][i]
                break
        return keyframe

    def get_previous_keyframe(self, track, frame):
        track = self.tracks[track]
        keyframe = track['frames'][-1]
        for i in xrange(len(track['frames'])):
            if frame < track['frames'][i][0]:
                if i != 0:
                    keyframe = track['frames'][i - 1]
                break
        return keyframe
