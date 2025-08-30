FROM ros:humble

RUN apt update && apt install -y iputils-ping net-tools nano gedit x11-apps mlocate python3-pip curl wget sudo

#Configure Timezone
RUN apt-get update && \
    apt-get install -yq tzdata && \
    ln -fs /usr/share/zoneinfo/Europe/Rome /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

RUN pip install -U pip 
RUN apt update && apt install -y ros-humble-rviz2 \
                                ros-humble-rqt-common-plugins \
                                ros-humble-rqt-tf-tree \
                                ros-humble-rqt-image-view

WORKDIR /root/scripts                        
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

#NAOqi
ADD install_qi.bash /root/install_qi.bash
RUN chmod +x /root/install_qi.bash

#Installing GUI
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install xfce4 xfce4-goodies -y
RUN apt update && apt install xrdp -y
RUN update-alternatives --set x-terminal-emulator /usr/bin/xfce4-terminal.wrapper
RUN adduser xrdp ssl-cert 

#User Creation
RUN useradd -m -s /bin/bash mivia \
    && echo 'mivia:mivia' | chpasswd \
    && usermod -aG sudo,dialout mivia 
RUN echo "mivia ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

#Colcon utils
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> /root/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

USER mivia
WORKDIR /home/mivia/
ADD --chown=mivia run_xrdp.bash .
RUN chmod +x run_xrdp.bash
RUN echo "xfce4-session" | tee .xsession

RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc

# RDP
EXPOSE 3389

ENV ROS_LOCALHOST_ONLY=1
ENV ROS_DOMAIN_ID=0
ENTRYPOINT ["bash", "/home/mivia/run_xrdp.bash" ]