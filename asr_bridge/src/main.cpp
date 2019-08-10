#include <ros/ros.h>
#include <audio_common_msgs/AudioData.h>

#include <hirop/asr/audio_source.h>
#include <hirop/asr/hirop_asr.h>

#include <alsa/asoundlib.h>
#include <std_msgs/String.h>

#include <hirop_msgs/StartListen.h>
#include <hirop_msgs/StopListen.h>

using namespace HIROP::ASR;

class RosAudioSource : public GetIntentListener{

private:
    snd_pcm_t *whandle;

    snd_pcm_hw_params_t *wparams;

    snd_pcm_uframes_t frames;

    unsigned int val;

    int rc;

    int wdir;

    ros::Publisher intentPusher;

    ros::NodeHandle _n;

    ros::ServiceServer startServer;

    ros::ServiceServer stopServer;

public:
    RosAudioSource(){

        hasr = HSpeechRecognition::getInstance();

        intentPusher = _n.advertise<std_msgs::String>("/user_intent", 1);

        startServer = _n.advertiseService("/start_listen", &RosAudioSource::onStart, this);

        stopServer = _n.advertiseService("/stop_listen", &RosAudioSource::onStop, this);

        hasr->init();

        hasr->setIntentListener(this);

        initALSA();

    }

    int updateAudioData(char *buf){

        return 0;
    }

    void wirteAndRead(){

        int size = frames * 2;

        char *buffer;
        buffer = ( char * ) malloc(size);

        while(!ros::isShuttingDown()){
            rc = snd_pcm_readi(whandle,buffer,frames);
            if ( rc == -EPIPE )
            {
                fprintf(stderr,"overrun occured\n");
                snd_pcm_prepare(whandle);
            }
            else if ( rc < 0 )
            {
                fprintf(stderr,"error from read: %s\n",
                        snd_strerror(rc));
            }
            else if ( rc != (int)frames)
            {
                fprintf(stderr,"short read, read %d frames\n",rc);
            }

            hasr->updateAudioData(buffer, size);

        }

    }

    void initALSA(){

        rc = snd_pcm_open(&whandle, "default", SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0) {
            fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(rc));
            exit(1);
        }

        /* Allocate a hardware parameters object. */
        snd_pcm_hw_params_alloca(&wparams);

        /* Fill it in with default values. */
        snd_pcm_hw_params_any(whandle, wparams);

        snd_pcm_hw_params_set_access(whandle, wparams, SND_PCM_ACCESS_RW_INTERLEAVED);

        // 位数
        snd_pcm_hw_params_set_format(whandle, wparams, SND_PCM_FORMAT_S16);

        // 通道数
        snd_pcm_hw_params_set_channels(whandle, wparams, 1);

        // 采用率
        val = 16000;
        snd_pcm_hw_params_set_rate_near(whandle, wparams, &val, &wdir);

        // 一帧数据的大小
        frames = 16;
        snd_pcm_hw_params_set_period_size_near(whandle, wparams, &frames, &wdir);

        rc = snd_pcm_hw_params(whandle, wparams);
        if (rc < 0) {
            fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
            exit(1);
        }

    }

    void onGetIntent(const char* intent){

        std::cout << intent << std::endl;

        std_msgs::String msg;
        std::string str(intent);

        msg.data = str;
        intentPusher.publish(msg);

    }

    bool onStart(hirop_msgs::StartListen::Request &req, hirop_msgs::StartListen::Response &rep){

        std::cout << "start listen" << std::endl;

        hasr->startListen();

        rep.reuslt = 0;

        return true;
    }

    bool onStop(hirop_msgs::StopListen::Request &req, hirop_msgs::StopListen::Response &rep){

        std::cout << "stop listen" << std::endl;

        hasr->stopListen();

        rep.reuslt = 0;

        return true;
    }

private:
    HSpeechRecognition *hasr;

};

int main(int argc, char* argv[]){

    ros::init(argc, argv, "asr_bridge");
    ros::NodeHandle n;

    RosAudioSource *as = new RosAudioSource();


    ros::AsyncSpinner spinner(2);
    spinner.start();

    as->wirteAndRead();


    ros::waitForShutdown();

    return 0;
}
