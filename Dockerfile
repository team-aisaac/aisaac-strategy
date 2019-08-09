FROM osrf/ros:melodic-desktop-full

RUN apt-get update -y
RUN apt-get install -y python-tk libprotobuf-dev libprotoc-dev protobuf-compiler python-pip ros-melodic-navigation ros-melodic-bfl graphviz libgraphviz-dev pkg-config psmisc

RUN pip2 install protobuf
RUN pip2 install pygraphviz --install-option="--include-path=/usr/include/graphviz" --install-option="--library-path=/usr/lib/graphviz/"

RUN apt-get -y clean

COPY . /aisaac-strategy
RUN pip2 install -r /aisaac-strategy/requirements.txt

COPY ./runtest.sh /runtest.sh
CMD bash /runtest.sh
