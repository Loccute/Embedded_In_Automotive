# Embedded In Automotive

<details>
	<summary><strong>BÀI 1: SETUP PROJECT ĐẦU TIÊN TRÊN KEILC</strong></summary>

## BÀI 1: SETUP PROJECT ĐẦU TIÊN TRÊN KEILC
### 1. KeilC:
- Phần mềm được phát triển bởi công ti ARM.
- Tạo môi trường tạo ra để lập trình các ngôn ngữ C và Assembly. Có thể biên dịch các chương  
- giúp biên dịch chương trình C/Assembly thành mã máy (.hex file) để máy tính có thể hiểu được và nạp vào các vi điều khiển.

### 2. Tạo project đầu tiên với KeilC
- Cần các thiết bị: STM32, ST-Link Driver
- Cài đặt thư viện chuẩn cho STM32, Tải Datasheet, Reference Manual.
- Các bước tạo project mới trên KeilC, thêm file và thư viện cần thiết. Các thao tác cơ bản trên KeilC (Build, Nạp, Debug Code,...).
</details>


<details>
	<summary><strong>BÀI 2: GPIO</strong></summary>
  
## BÀI 2: GPIO

 

 </details>


<details>
	<summary><strong>BÀI 3: NGẮT VÀ TIMER</strong></summary>
  
## BÀI 3: NGẮT VÀ TIMER
### 1. Ngắt
- Ngắt là 1 sự kiện khẩn cấp xảy ra trong hay ngoài vi điều khiển. Nó yêu cầu MCU phải dừng chương trình chính và thực thi chương trình ngắt (trình phục vụ ngắt).
- Trình phục vụ ngắt (Interrupt Service Routine - ISR) là một đoạn chương trình được thực hiện khi ngắt xảy ra. Địa chỉ trong bộ nhớ của ISR là "Vector ngắt".
- Có 4 loại ngắt thông dụng:
  + Ngắt ngoài: Xảy ra khi có sự thay đổi điện áp ở các chân GPIO cấu hình làm ngõ vào ngắt. Gồm 4 dạng: HIGH, LOW, RISING, FALLING tương ứng với các mức và sự thay đổi lên xuống của điện áp.
  + Ngắt timer: Xảy ra khi thanh ghi trong bộ đếm Timer bị tràn, khi đó giá trị thanh ghi sẽ bị reset để tạo ngắt tiếp theo.
  + Ngắt truyền thông: Xảy ra khi có sự kiện truyền thông (truyền và nhận dữ liệu) từ 2 hay nhiều thiết bị. Thường sử dụng cho các giao thức như SPI,UART, I2C để truyền nhận dữ liệu chính xác.
- Thanh ghi PC: là thanh ghi trỏ tới địa chỉ lệnh tiếp theo được thực thi.
- Độ ưu tiên ngắt: các ngắt có độ ưu tiên khác nhau. Trên STM32, số thứ tự ngắt càng thấp thì ngắt càng được ưu tiên Độ ưu tiên ngắt có thể lập trình được.

### 2. Timer
- Là 1 mạch logic được thiết kế trong STM32 dùng để đếm chu kì xung clock.
- STM32F103 có 7 timer.
- Cấu hình cho Timer: Ta cấu hình các thành phần trong struct TIM_TimeBaseInitTypeDef
  + TIM_Prescaler: Cấu hình bộ chia tầng số, quy định sau bao nhiêu xung clk thì đếm lên 1 lần.
  + TIM_CounterMode: Chỉ định chế độ đếm là đếm lên hay đếm xuống (TIM_CounterMode_Up: đếm lên, TIM_CounterMode_Down: đếm xuống).
  + TIM_Period: Chỉ định 1 chu kì của bộ đếm (đếm đến bao nhiêu xung thì reset).
  + TIM_ClockDivision: Cấu hình bộ chia xung (thường dùng TIM_CKD_DIV1: chia cho 1), (fTimer = fSystem/TIM_ClockDivision; trong đó fSystem: tần số hệ thống - 72MHz, fTimer: Tần số của Timer).
   
Ví dụ:

```c
// Đầu tiên, ta cần cấp clk cho Timer hoạt động
// Ta dùng TIM2
void RCC_Config(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

// Cấu hình cho TIMER
void TIM_Config(){
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;     // fTimer = 72MHz
	TIM_InitStruct.TIM_Prescaler = 7200 - 1;	      // 7200 xung clk thì đếm lên 1 lần --> sau 7200/fTimer = 7200/72000000 = 0.1 (ms)
	TIM_InitStruct.TIM_Period = 0xFFFF;                  // Chu kì reset: 0xFFFF - 65535
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up; // Chế độ đếm lên
	
	TIM_TimeBaseInit(TIM2, &TIM_InitStruct);             // Hàm lưu cấu hình của Timer
	TIM_Cmd(TIM2, ENABLE);                               // Bật timer 2
}
```

Ngoài ra còn 2 hàm:
```c
void TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter);
// Đặt giá trị ban đầu cho timer

uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);
// Lấy giá trị đếm hiện tại của timer
```

Ví dụ: Ta thiết kế hàm delay_ms, tham số truyền vào là thời gian delay (đơn vị ms)

```c
void delay_ms(uint32_t time){
	TIM_SetCounter(TIM2, 0);
	while(TIM_GetCounter(TIM2) < time * 10){}
}

``` 

 </details>
