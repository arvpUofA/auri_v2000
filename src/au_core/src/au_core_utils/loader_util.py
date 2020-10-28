import os
import yaml
import rospkg


def __load_yaml(f):
    rospack = rospkg.RosPack()
    path = os.path.join(rospack.get_path('au_core'), 'params', f)
    data = None
    try:
        with open(path, 'r') as f:
            data = yaml.load(f)
    except:
        raise RuntimeError('Unable to open yaml file: ' + path)
    return data


def __static_vars(**kwargs):
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func

    return decorate


@__static_vars(topics=__load_yaml('topics.yaml'))
def load_topic(key):
    return load_topic.topics[key]


@__static_vars(frames=__load_yaml('frames.yaml'))
def load_frame(key):
    frame_path = key.strip().split('/')
    frame = load_frame.frames
    for f in frame_path:
        frame = frame[f]
    return frame


@__static_vars(frames=__load_yaml('scene_objects.yaml'))
def load_all_objects():
    return load_object.frames


@__static_vars(frames=__load_yaml('scene_objects.yaml'))
def load_object(name):
    return load_object.frames[name]
