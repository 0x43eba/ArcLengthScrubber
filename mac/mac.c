#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <AudioToolbox/AudioToolbox.h>
#include <pthread.h>

#define DEVICE "/dev/cu.usbmodem*"  // macOS Arduino device pattern
#define SAMPLE_RATE 44100
#define BUFFER_SIZE 512
#define START_BYTE 0xAA

#pragma pack(push, 1)
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} SensorDataOutput;
#pragma pack(pop)

typedef struct {
    double phase;
    double frequency;
    double volume;
    pthread_mutex_t mutex;
} AudioData;

AudioData g_audio_data = {0.0, 440.0, 0.3, PTHREAD_MUTEX_INITIALIZER};

int open_serial(const char* path) {
    int fd = open(path, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("open_serial");
        exit(1);
    }

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);

    return fd;
}

double angle_to_freq(double angle) {
    double norm = (angle + M_PI) / (2 * M_PI);
    return 220.0 + norm * (880.0 - 220.0);
}

OSStatus audio_callback(void *inRefCon,
                       AudioUnitRenderActionFlags *ioActionFlags,
                       const AudioTimeStamp *inTimeStamp,
                       UInt32 inBusNumber,
                       UInt32 inNumberFrames,
                       AudioBufferList *ioData) {
    
    AudioData *audio_data = (AudioData *)inRefCon;
    Float32 *buffer = (Float32 *)ioData->mBuffers[0].mData;
    
    pthread_mutex_lock(&audio_data->mutex);
    double phase = audio_data->phase;
    double freq = audio_data->frequency;
    double volume = audio_data->volume;
    pthread_mutex_unlock(&audio_data->mutex);
    
    double phase_step = 2.0 * M_PI * freq / SAMPLE_RATE;
    
    for (UInt32 i = 0; i < inNumberFrames; i++) {
        buffer[i] = volume * sin(phase);
        phase += phase_step;
        if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
    }
    
    pthread_mutex_lock(&audio_data->mutex);
    audio_data->phase = phase;
    pthread_mutex_unlock(&audio_data->mutex);
    
    return noErr;
}

AudioUnit init_core_audio() {
    AudioComponentDescription desc;
    desc.componentType = kAudioUnitType_Output;
    desc.componentSubType = kAudioUnitSubType_DefaultOutput;
    desc.componentManufacturer = kAudioUnitManufacturer_Apple;
    desc.componentFlags = 0;
    desc.componentFlagsMask = 0;
    
    AudioComponent component = AudioComponentFindNext(NULL, &desc);
    if (!component) {
        fprintf(stderr, "Failed to find audio component\n");
        exit(1);
    }
    
    AudioUnit audio_unit;
    OSStatus status = AudioComponentInstanceNew(component, &audio_unit);
    if (status != noErr) {
        fprintf(stderr, "Failed to create audio unit: %d\n", (int)status);
        exit(1);
    }
    
    // Set up audio format
    AudioStreamBasicDescription format;
    format.mSampleRate = SAMPLE_RATE;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
    format.mBytesPerPacket = sizeof(Float32);
    format.mFramesPerPacket = 1;
    format.mBytesPerFrame = sizeof(Float32);
    format.mChannelsPerFrame = 1;
    format.mBitsPerChannel = 32;
    
    status = AudioUnitSetProperty(audio_unit,
                                 kAudioUnitProperty_StreamFormat,
                                 kAudioUnitScope_Input,
                                 0,
                                 &format,
                                 sizeof(format));
    if (status != noErr) {
        fprintf(stderr, "Failed to set audio format: %d\n", (int)status);
        exit(1);
    }
    
    // Set callback
    AURenderCallbackStruct callback_struct;
    callback_struct.inputProc = audio_callback;
    callback_struct.inputProcRefCon = &g_audio_data;
    
    status = AudioUnitSetProperty(audio_unit,
                                 kAudioUnitProperty_SetRenderCallback,
                                 kAudioUnitScope_Input,
                                 0,
                                 &callback_struct,
                                 sizeof(callback_struct));
    if (status != noErr) {
        fprintf(stderr, "Failed to set callback: %d\n", (int)status);
        exit(1);
    }
    
    // Initialize and start
    status = AudioUnitInitialize(audio_unit);
    if (status != noErr) {
        fprintf(stderr, "Failed to initialize audio unit: %d\n", (int)status);
        exit(1);
    }
    
    status = AudioOutputUnitStart(audio_unit);
    if (status != noErr) {
        fprintf(stderr, "Failed to start audio unit: %d\n", (int)status);
        exit(1);
    }
    
    return audio_unit;
}

char* find_arduino_device() {
    static char device_path[256];
    char command[] = "ls /dev/cu.usbmodem* 2>/dev/null | head -1";
    FILE *fp = popen(command, "r");
    
    if (fp == NULL || fgets(device_path, sizeof(device_path), fp) == NULL) {
        pclose(fp);
        fprintf(stderr, "Arduino device not found. Make sure it's connected.\n");
        exit(1);
    }
    
    pclose(fp);
    
    // Remove newline
    device_path[strcspn(device_path, "\n")] = 0;
    return device_path;
}

int main() {
    char *device_path = find_arduino_device();
    printf("Using Arduino device: %s\n", device_path);
    
    int serial = open_serial(device_path);
    AudioUnit audio_unit = init_core_audio();

    uint8_t b;
    SensorDataOutput data;

    printf("Listening for sensor data...\n");

    while (1) {
        if (read(serial, &b, 1) != 1 || b != START_BYTE) continue;

        ssize_t n = read(serial, &data, sizeof(SensorDataOutput));
        if (n != sizeof(SensorDataOutput)) continue;

        double angle = atan2((double)data.y, (double)data.x);
        double freq = angle_to_freq(angle);

        // Update audio parameters thread-safely
        pthread_mutex_lock(&g_audio_data.mutex);
        g_audio_data.frequency = freq;
        pthread_mutex_unlock(&g_audio_data.mutex);
        
        printf("Angle: %.2f, Frequency: %.1f Hz\n", angle, freq);
    }

    AudioOutputUnitStop(audio_unit);
    AudioUnitUninitialize(audio_unit);
    AudioComponentInstanceDispose(audio_unit);
    close(serial);
    pthread_mutex_destroy(&g_audio_data.mutex);
    
    return 0;
}
