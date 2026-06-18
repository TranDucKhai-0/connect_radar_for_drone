#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    // Khởi tạo socket RAW cho CAN
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Tiểu Đệ không mở được socket");
        return 1;
    }

    // Chỉ định interface là can0
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind socket vào can0
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Tiểu Đệ không bind được vào can0");
        return 1;
    }

    printf("Tiểu Đệ đang lắng nghe dữ liệu MR72 trên can0 (Nhấn Ctrl+C để thoát)...\n");

    // Vòng lặp đọc dữ liệu vắt kiệt hiệu suất
    while (1) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Lỗi đọc dữ liệu");
            break;
        }

        // Lọc và xử lý bản tin trạng thái (Radar ID = 1 -> 0x221)
        if (frame.can_id == 0x221) {
            printf("\n[0x221] Nhận được cấu hình (RadarState)\n");
            
            /* * Bóc tách MaxDistanceCfg [cite: 237]
             * - Vị trí: Start bit 22, dài 10 bit.
             * - Dữ liệu nằm vắt qua Byte 1 (8 bit cao) và Byte 2 (2 bit cao nhất).
             * - Độ phân giải (Res) = 2 mét.
             * - Công thức: Gộp Byte 1 và Byte 2, sau đó nhân với 2[cite: 510].
             */
            int max_distance = ((frame.data[1] << 2) | (frame.data[2] >> 6)) * 2;
            printf("  -> Cấu hình Max Distance: %d mét\n", max_distance);

            /* * Bóc tách RadarPowerCfg [cite: 237]
             * - Vị trí: Start bit 39, dài 3 bit.
             * - Dữ liệu nằm ở 3 bit cao nhất của Byte 4 (bit 39, 38, 37).
             * - Dịch phải 5 bit để đẩy 3 bit này xuống cuối, rồi AND với 0x07 (00000111) để lọc.
             */
            int power_cfg = (frame.data[4] >> 5) & 0x07;
            printf("  -> Cấu hình Radar Power: ");
            switch(power_cfg) {
                case 0: printf("Standard (Chuẩn)\n"); break;
                case 1: printf("-3dB Tx Gain\n"); break;
                case 2: printf("-6dB Tx Gain\n"); break;
                case 3: printf("-9dB Tx Gain\n"); break;
                default: printf("Unknown (Không xác định)\n");
            }
        }
        
        // Lọc và xử lý bản tin số lượng mục tiêu (Radar ID = 1 -> 0x61A)
        else if (frame.can_id == 0x62A) {
            // Objects_NofObjects (Độ dài 8 bit) bắt đầu ở Byte 0 [cite: 393]
            printf("\n[0x62A] Frame mới - Số lượng Point phát hiện được: %d\n", frame.data[0]);
        }
    }

    close(s);
    return 0;
}