#/bin/bash

#RCIS
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis /home/salabeta/robocomp/files/innermodel/simpleworld.xml'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis'
sleep 1



#joystickComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '/home/salabeta/robocomp/components/robocomp-robolab/components/joystickComp/bin/joystickComp --Ice.Config=/home/salabeta/robocomp/components/robocomp-robolab/components/joystickComp/etc/config'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joystick'
sleep 1

# Ice Storm
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcnode'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'storm'
sleep 1


# AprilTags
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '/home/salabeta/robocomp/components/robocomp-robolab/components/apriltagsComp/bin/apriltagscomp --Ice.Config=/home/salabeta/robocomp/components/robocomp-robolab/components/apriltagsComp/etc/config'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'april'
sleep 1



