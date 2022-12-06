FROM hanlinwang/coachswarm:swarmclass
MAINTAINER Hanlin Wang <hanlinwang@u.northwestern.edu>

WORKDIR /usr/CoachSwarm
RUN rm -f *.pyc
COPY fs/usr_code.py /usr/CoachSwarm/
COPY fs/coachswarm.conf /usr/CoachSwarm/
COPY fs/init_pose.py /usr/CoachSwarm/
RUN chmod 777 /usr/CoachSwarm/*

#ADD fs .



