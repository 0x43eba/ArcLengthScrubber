#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <alsa/asoundlib.h>
#include <string.h>

#define DEVICE "/dev/ttyACM0"
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

void fill_sine_wave(int16_t *buffer, int samples, double *phase, double freq, double volume) {
    double phase_step = 2.0 * M_PI * freq / SAMPLE_RATE;
    for (int i = 0; i < samples; i++) {
        buffer[i] = (int16_t)(volume * 32767.0 * sin(*phase));
        *phase += phase_step;
        if (*phase >= 2.0 * M_PI) *phase -= 2.0 * M_PI;
    }
}

snd_pcm_t* init_alsa() {
    snd_pcm_t *handle;
    snd_pcm_hw_params_t *params;
    int dir;

    if (snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        perror("ALSA open");
        exit(1);
    }

    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(handle, params);
    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(handle, params, 1);
    snd_pcm_hw_params_set_rate_near(handle, params, (unsigned int[]){SAMPLE_RATE}, &dir);
    snd_pcm_hw_params(handle, params);
    snd_pcm_prepare(handle);

    return handle;
}

int main() {
    int serial = open_serial(DEVICE);
    snd_pcm_t *pcm = init_alsa();

    uint8_t b;
    SensorDataOutput data;
    int16_t wave[BUFFER_SIZE];
    double phase = 0.0;

    while (1) {
        if (read(serial, &b, 1) != 1 || b != START_BYTE) continue;

        ssize_t n = read(serial, &data, sizeof(SensorDataOutput));
        if (n != sizeof(SensorDataOutput)) continue;

        double angle = atan2((double)data.y, (double)data.x);
        double freq = angle_to_freq(angle);

        fill_sine_wave(wave, BUFFER_SIZE, &phase, freq, 0.3);

        if (snd_pcm_writei(pcm, wave, BUFFER_SIZE) < 0) {
            snd_pcm_prepare(pcm);
        }
    }

    close(serial);
    snd_pcm_close(pcm);
    return 0;
}
