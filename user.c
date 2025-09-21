/* -------------------- user_dashboard.c -------------------- */
/* Lecture en boucle de TMP102, BMP280 et ADS1115 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

struct bmp_user_out {
    int32_t temperature_centi;
    int32_t pressure_pa;
};

int main() {
    int fd_tmp, fd_bmp, fd_ads;
    short tmp_raw;
    struct bmp_user_out bmp_data;
    int16_t ads_raw;

    // --- Ouverture des périphériques ---
    fd_tmp = open("/dev/tmp102", O_RDWR);
    if (fd_tmp < 0) {
        perror("Erreur ouverture TMP102");
        return 1;
    }

    fd_bmp = open("/dev/bmp280", O_RDWR);
    if (fd_bmp < 0) {
        perror("Erreur ouverture BMP280");
        return 1;
    }

    fd_ads = open("/dev/ads1115", O_RDWR);
    if (fd_ads < 0) {
        perror("Erreur ouverture ADS1115");
        return 1;
    }

    while (1) {
        // --- Lire TMP102 ---
        if (read(fd_tmp, &tmp_raw, sizeof(tmp_raw)) > 0) {
            float temp_C = (tmp_raw & 0x0FFF) * 0.0625; // TMP102: 12-bit LSB = 0.0625°C
            printf("TMP102: raw = 0x%04x -> Temp = %.2f °C\n", tmp_raw & 0x0FFF, temp_C);
        }

        // --- Lire BMP280 ---
        if (read(fd_bmp, &bmp_data, sizeof(bmp_data)) > 0) {
            float temp_C = bmp_data.temperature_centi / 100.0;
            float pres_hPa = bmp_data.pressure_pa / 100.0;
            printf("BMP280: Temp = %.2f °C, Pressure = %.2f hPa\n", temp_C, pres_hPa);
        }

        // --- Lire ADS1115 ---
        if (read(fd_ads, &ads_raw, sizeof(ads_raw)) > 0) {
            float voltage = ads_raw / 32768.0 * 3.3; // Conversion en volts
            printf("ADS1115: raw = %d -> Voltage = %.3f V\n", ads_raw, voltage);
        }

        printf("-------------------------------\n");
        usleep(500000); // 500 ms
    }

    close(fd_tmp);
    close(fd_bmp);
    close(fd_ads);

    return 0;
}
