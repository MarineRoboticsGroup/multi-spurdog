import importlib
import pkgutil
import os
import rospy
import sys
from typing import Union, List

default_codecs = {
    name.split(".")[-1].split("_")[0]: importlib.import_module(name)
    for finder, name, ispkg in pkgutil.iter_modules(__path__, __name__ + ".")
    if name.endswith("_packet_codec") and name != "acomms_codecs.base_packet_codec"
}

packet_codecs = {**default_codecs}

def load_custom_codecs(custom_packet_codec_paths: Union[str, List[str]]) -> None:
    """
    Loads custom packet codecs from the specified path(s).

    The codecs are loaded directly into the packet_codecs dictionary.

    :param custom_packet_codec_paths: List of paths or string with single path.  These should be directory names with no trailing slash.
    """
    custom_codecs = {}

    if custom_packet_codec_paths is not None:
        try:
            custom_codecs = {}

            for codec_path in (
                custom_packet_codec_paths
                if isinstance(custom_packet_codec_paths, list)
                else [custom_packet_codec_paths]
            ):
                for finder, name, ispkg in pkgutil.walk_packages(path=[codec_path]):
                    if name.endswith("_packet_codec"):
                        codec_name = name.split(".")[-1].split("_")[0]
                        codec = finder.find_module(name).load_module(name)
                        custom_codecs[codec_name] = codec

        except OSError as exception:
            rospy.logerr(
                f"OSError occured when loading custom packet codecs, check provided path: {exception}"
            )
            raise OSError(
                "OSError occured when loading custom packet codecs, check provided path"
            ) from exception
        except ImportError as exception:
            rospy.logerr(
                f"ImportError occured when loading custom packet codecs: {exception}"
            )
            raise ImportError(
                "ImportError occured when loading custom packet codecs"
            ) from exception
        except Exception as exception:
            if codec:
                rospy.logerr(
                    f"An unhandled error occured while loading custom packet codecs from {codec}: {exception}."
                )
            else:
                rospy.logerr(
                    f"An unhandled error occured while loading custom packet codecs: {exception}."
                )
            raise Exception(
                "An unhandled error occured while loading custom packet codecs"
            ) from exception
    else:
        rospy.loginfo("No custom packet codecs specified, using defaults")

    this = sys.modules[__name__]
    this.packet_codecs.update(**custom_codecs)


