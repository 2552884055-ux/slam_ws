#include "tf_publish_node.hpp"

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tf_publish");
    TfPublish Tf_pub;

    if(Tf_pub.socketEn_get()){
        std::thread socketServerThread(&TfPublish::socketServer,&Tf_pub);
        socketServerThread.detach();
    }

    std::thread mainThread(&TfPublish::run,&Tf_pub);
    mainThread.detach();

    ros::spin();
    return 0;
}
