#!/usr/bin/python3
# coding = utf-8


# voicevox imports
import core

# audio imports
import simpleaudio as sa

# ROS imports
import rospy
from orne_voicevox_core.srv import get_speak_paramater, get_speak_paramaterResponse


class speakNode():
    def __init__(self):
        # Service info
        self.service_name = rospy.get_param("~service_name", "speak")
        self.speak_service = rospy.Service(self.service_name, get_speak_paramater, self.callback_srv)
        rospy.loginfo("service name: %s" % self.service_name)

        # voice_core info
        self.cpu_num_threads = rospy.get_param("~cpu_num_threads", 0)
        rospy.loginfo("cpu_num_threads: %d" % self.cpu_num_threads)

        self.openjtalk_dic_path = rospy.get_param("~openjtalk_dic_path", "open_jtalk_dic_utf_8-1.11")
        rospy.loginfo("openjtalk_dic_path: %s" % self.openjtalk_dic_path)

        self.use_gpu = rospy.get_param("~use_gpu", False)
        rospy.loginfo("use_gpu: %s" % self.use_gpu)

        # コアの初期化
        rospy.loginfo("コア初期化開始")
        core.initialize(self.use_gpu, self.cpu_num_threads)
        rospy.loginfo("コア初期化完了")

        # openjtalk辞書のロード
        rospy.loginfo("openjtalk辞書ロード開始")
        core.voicevox_load_openjtalk_dict(self.openjtalk_dic_path)
        rospy.loginfo("openjtalk辞書ロード完了")
        rospy.loginfo("サービス準備完了")

    def callback_srv(self, req):
        text_inp = req.text
        speaker_id_inp = req.speaker_id

        # 音声合成
        rospy.loginfo("\"%s\"合成開始" % text_inp)
        wavefmt = core.voicevox_tts(text_inp, speaker_id_inp)
        rospy.loginfo("\"%s\"合成完了" % text_inp)
        rospy.loginfo("再生中")
        wave_obj = sa.WaveObject(wavefmt, 1, 2, 24000)
        play_obj = wave_obj.play()
        play_obj.wait_done()
        rospy.loginfo("再生完了")

        return get_speak_paramaterResponse()

    # 保存
    #     with open(f"{strline_inp}.wav", "wb") as f:
    #         f.write(wavefmt))
    #         print("保存完了")


if __name__ == '__main__':
    rospy.init_node('orne_voicevox_engine')
    while not rospy.is_shutdown():
        speakNode()
        rospy.spin()

    core.finalize()
