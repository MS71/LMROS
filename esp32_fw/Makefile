PROJECT_NAME := ros2node

include $(IDF_PATH)/make/project.mk

doota:  
	curl 192.168.0.154:8032 --data-binary @- < build/$(PROJECT_NAME).bin

udplog:
	nc -u -l 30000
