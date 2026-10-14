
gcc -shared -fPIC \
    -g -O0 \
    -DRING_BUFFER_SIZE=8192 \
    -DEXTRACTION_BUFFER_SIZE=256 \
    -I/home/seb/nas/docs/projects/hand/hand_firmware/Core/Inc \
    -o bin/message_protocol.so \
    /home/seb/nas/docs/projects/hand/hand_firmware/Core/Src/message_protocol.c
