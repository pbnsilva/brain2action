from utils import Quaternion


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
