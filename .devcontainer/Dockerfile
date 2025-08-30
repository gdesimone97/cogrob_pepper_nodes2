FROM ros:humble

RUN apt update && apt install -y net-tools nano gedit x11-apps mlocate python3-pip curl wget
RUN pip install -U pip 
RUN apt update && apt install -y ros-humble-rviz2 \
                                ros-humble-rqt-common-plugins \
                                ros-humble-rqt-tf-tree \
                                ros-humble-rqt-image-view

WORKDIR /root/scripts                        
ADD install_qi.bash install_qi.bash
RUN bash install_qi.bash install_qi.bash
RUN pip install pyOpenssl -U

#Installing RASA
WORKDIR /root
RUN pip install rasa==3.6.21 rasa[spacy]
RUN python3 -m spacy download en_core_web_md
RUN python3 -m spacy download it_core_news_md

#CV bridge need numpy<2
RUN pip install -U 'numpy<2'

#Fix Pepper 230
WORKDIR /usr/local/bin
RUN apt update && apt install -y sshpass
RUN wget https://github.com/gdesimone97/cogrob_pepper_nodes/releases/download/fix-pepper/fix_pepper 
RUN chmod +x fix_pepper

WORKDIR /root
RUN ln -s /usr/bin/python3 /usr/local/bin/python

#Colcon utils
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> /root/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc