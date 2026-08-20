"""Vendored from giava@real-v2-spr26, interbotix_ws/src/av_aloha/data_collection_scripts.

Copied rather than submoduled because upstream is a ROS 1 catkin workspace whose
package layout, dataset paths and rospy entry points do not survive the move to
ROS 2 -- but these four files are pure Python and do survive it intact.

    webrtc_headset.py   WebRTCHeadset: aiortc + Firestore signalling, the
                        "control" data channel, stereo video back to the headset
    headset_utils.py    HeadsetData/HeadsetFeedback, Unity <-> ROS conversion
    transform_utils.py  pose/quaternion math
    teleop_map.py       the controller->arm mapping (see its docstring for the
                        changes made and the two constants that do not transfer)

Changes are confined to import lines and the secret-file paths; the maths is
untouched, so upstream fixes can be diffed straight in. Re-extract with:

    cd ~/giava && git show origin/real-v2-spr26:interbotix_ws/src/av_aloha/\\
        data_collection_scripts/<file>
"""
