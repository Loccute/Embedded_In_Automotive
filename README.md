# Embedded In Automotive

<details>
	<summary><strong>BÀI 1: SETUP PROJECT ĐẦU TIÊN TRÊN KEILC</strong></summary>

## BÀI 1: SETUP PROJECT ĐẦU TIÊN TRÊN KEILC
### 1. KeilC:
- Phần mềm được phát triển bởi công ti ARM.
- Tạo môi trường tạo ra để lập trình các ngôn ngữ C và Assembly. Có thể biên dịch các chương  
- Giúp biên dịch chương trình C/Assembly thành mã máy (.hex file) để máy tính có thể hiểu được và nạp vào các vi điều khiển.

### 2. Tạo project đầu tiên với KeilC
- Cần các thiết bị: STM32, ST-Link Driver
- Cài đặt thư viện chuẩn cho STM32, Tải Datasheet, Reference Manual.
- Các bước tạo project mới trên KeilC, thêm file và thư viện cần thiết. Các thao tác cơ bản trên KeilC (Build, Nạp, Debug Code,...).

### 3. Ví dụ Blink LED PC13
- Các bước thực hiện:
  + Cấp xung clock cho ngoại vi.
  + Cấu hình chân của ngoại vi.
  + Sử dụng ngoại vi.
- Tổng hợp địa chỉ các thanh ghi: ![image](https://github.com/user-attachments/assets/38423912-9bcc-4e0a-ab46-d0b449fa028f)
- B1: cấu hình thanh ghi RCC_APB2ENR để cấp clock cho ngoại vi:
  ![image](https://github.com/user-attachments/assets/ad4001c4-77c5-471e-9919-f072c1a13ad6)

  ```c
  #define RCC_APB2ENR	*((unsigned int *)0x40021018)
  RCC_APB2ENR |= (1 << 4); // Kich hoat xung clock cap cho GPIOC
  ```
- B2: Cấu hình chế độ chân PC13: Ta dặt chế độ là output push-pull có điện trở kéo lên
  ![image](https://github.com/user-attachments/assets/0517277f-a495-44c5-bcdd-ebea2092fc17)

  ```c
  #define GPIOC_CRH	*((unsigned int *)0x40011004)
  // MODE13[1:0] = 11: Output mode, max speed 50 MHz	
  GPIOC_CRH |= (1 << 20) | (1 << 21);

  // CNF13[1:0] = 00: General purpose output push-pull
  GPIOC_CRH &= ~((1 << 22) | (1 << 23));
  ```
- B3: Sử dụng ngoại vi: Ta lần lượt ghi điện áp ở chân PC13 là 1, 0 xen kẽ nhau sau khi delay 1 khoảng thời gian để blink led PC13. Ta sẽ thao tác ghi mức điện áp trên thanh ghi ODR.
  ```c
  #define GPIOC_ODR *((unsigned int*)0x4001100C)
  while(1){
	GPIOC_ODR |= 1 << 13; // LED tắt
	delay(10000000);
	GPIOC_ODR &= ~(1 << 13); // LED sáng
	delay(10000000);
  }
  ```
  Ta sử dụng vòng lặp để tạo hàm delay
  ```c
  void delay(unsigned int timedelay){ 
	for(unsigned int i = 0; i < timedelay; i++){}
  }
  ```
- Ngoài ra, ta còn có thể xây dựng 1 cấu trúc thanh ghi của các ngoại vi để làm việc với ác ngoại vi được thuận tiện hơn:
  ```c
  typedef struct
  {
    unsigned int CRL;
    unsigned int CRH;
    unsigned int IDR;
    unsigned int ODR;
    unsigned int BSRR;
    unsigned int BRR;
    unsigned int LCKR;
  } GPIO_TypeDef;

  typedef struct
  {
    unsigned int CR;
    unsigned int CFGR;
    unsigned int CIR;
    unsigned int APB2RSTR;
    unsigned int APB1RSTR;
    unsigned int AHBENR;
    unsigned int APB2ENR;
    unsigned int APB1ENR;
    unsigned int BDCR;
    unsigned int CSR;
  } RCC_TypeDef;
  ```
### 4. Tổng kết và mở rộng:
- Code trên thanh ghi giúp ltv hiểu rõ cách hoạt động chi tiết của từng ngoại vi, tăng hiệu xuất chương trình.
- Nhưng lập trình thanh ghi có thể trở nên khá phức tạp đối với các hệ thống lớn.
- Nên sử dụng thư viện chuẩn của STM32 với các API có sẵn và dễ tiếp cận.

</details>


<details>
	<summary><strong>BÀI 2: GPIO</strong></summary>
  
## BÀI 2: GPIO
### 1. Thư viện STM32F10x Standard Peripherals Firmware Library
Là 1 thư viện hoàn chỉnh được phát triển cho dòng STM32. Bao gồm đầy đủ driver cho tất cả các ngoại vi tiêu chuẩn.

Thư viện này bao gồm các hàm, cấu trúc dữ liệu và macro của các tính năng thiết bị ngoại vi STM32. 

### 2. Cấu hình và sử dụng ngoại vi (GPIO)
- Gồm 3 bước cơ bản: cấp clock cho ngoại vi --> cấu hình ngoại vi --> sử dụng ngoại vi
- Ta sử dụng thư viện SPL là 1 thư viện chuẩn của STM32 cung cấp các hàm và các định nghĩa giúp việc cấu hình và sử dụng ngoại vi dễ dàng và rõ ràng.
#### 2.1 Cấp clock cho ngoại vi:
Ta dựa vào sơ đồ khối dưới đây để xác định đường bus phù hợp để cấp clock cho ngoại vi tương ứng: ![image](https://github.com/user-attachments/assets/a95e5397-0f2f-4043-b6ab-59422440586c)
Module RCC (Reset and Clock Control) cung cấp các hàm để cấu hình xung clock.
```c
RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)

RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
	
RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState)
```
- Các hàm này nhận 2 tham số: 
  + `RCC_APB1Periph`, `RCC_APB2Periph`, `RCC_AHBPeriph` là các ngoại vi muốn cấp clock. (Ví dụ
: RCC_APB2Periph_GPIOA, RCC_APB1Periph_CAN1,..)
  + `NewState` là giá trị quy định cấp (ENABLE) hay ngưng (DISABLE) xung clock cho ngoại vi đó.
#### 2.2 Cấu hình GPIO:
- Ta cấu hình các tham số cho GPIO được tổ chức trong struct GPIO_InitTypeDef:
  + `GPIO_Pin`: chọn chân muốn cấu hình,
  + `GPIO_Mode`: chọn chế độ của chân,
  + `GPIO_Speed`: chọn tốc độ chân.
- Có 8 chế độ của chân:

|Chế độ GPIO|Tên gọi|Mô tả|
|:----------|:------|:----|
|`GPIO_Mode_AIN`|**Analog Input**|Chân GPIO được cấu hình làm đầu vào analog. Thường được sử dụng cho các chức năng như ADC (Analog to Digital Converter).|
|`GPIO_Mode_IN_FLOATING`|**Floating Input**|Chân GPIO được cấu hình làm đầu vào và ở trạng thái nổi (không pull-up hay pull-down), nghĩa là chân không được kết nối cố định với mức cao (VDD) hoặc mức thấp (GND) thông qua điện trở.|
|`GPIO_Mode_IPD`|**Input Pull-Down**|Chân GPIO được cấu hình làm đầu vào với một điện trở pull-down nội bộ kích hoạt. Khi không có tín hiệu nào được áp dụng lên chân này, nó sẽ được kéo về mức thấp (GND).|
|`GPIO_Mode_IPU`|**Input Pull-Up**|Chân GPIO được cấu hình làm đầu vào với một điện trở pull-up nội bộ kích hoạt. Khi không có tín hiệu nào được áp dụng lên chân này, nó sẽ được kéo về mức cao (VDD).|
|`GPIO_Mode_Out_OD`|**Output Open-Drain**|Chân GPIO được cấu hình làm đầu ra với chế độ open-drain. Trong chế độ này, chân có thể được kéo xuống mức thấp, nhưng để đạt được mức cao, cần một điện trở pull-up ngoài hoặc từ một nguồn khác.|
|`GPIO_Mode_Out_PP`|**Output Push-Pull**|Chân GPIO được cấu hình làm đầu ra với chế độ push-pull. Trong chế độ này, chân có thể đạt được cả mức cao và mức thấp mà không cần bất kỳ phần cứng bổ sung nào.|
|`GPIO_Mode_AF_OD`|**Alternate Function Open-Drain**|Chân GPIO được cấu hình để hoạt động trong một chức năng thay thế (như USART, I2C,...) và sử dụng chế độ open-drain.|
|`GPIO_Mode_AF_PP`|**Alternate Function Push-Pull**|Chân GPIO được cấu hình để hoạt động trong một chức năng thay thế và sử dụng chế độ push-pull.|
  
- Có 3 mức tốc độ cho chân: GPIO_Speed_10MHz, GPIO_Speed_2MHz, GPIO_Speed_50MHz (Tốc độ nhanh nhất).
- Dùng hàm **GPIO_Init (GPIO_TypeDef, GPIO_InitStruct)** để khởi tạo GPIO:
  + `GPIO_TypeDef`: GPIO cần cấu hình
  + `GPIO_InitStruct`: Con trỏ trỏ tới biến TypeDef (Struct) vừa được khởi tạo

#### 2.3 Sử dụng ngoại vi:
Ta có 1 số hàm thông dụng để sử dụng ngoại vi

```c
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
\\Đọc giá trị 1 chân trong GPIO được cấu hình là INPUT
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
\\Đọc giá trị nguyên GPIO được cấu hình là INPUT
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
\\Đọc giá trị 1 chân trong GPIO được cấu hình là OUTPUT
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
\\Đọc giá trị nguyên GPIO được cấu hình là OUTPUT
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
\\Cho giá trị điện áp của 1 chân trong GPIO = 1
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
\\Cho giá trị điện áp của 1 chân trong GPIO = 0
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
\\Ghi giá trị "BitVal" vào 1 chân trong GPIO
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
\\Ghi giá trị "PortVal" vào nguyên GPIO

```

**Ví dụ 1**: Blink LED PC13
```c
while(1){
	GPIO_SetBits(GPIOC, GPIO_Pin_13); // Ghi 1 ra PC13
	delay(10000000);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);// Ghi 0 ra PC13
	delay(10000000);
}
```
**Ví dụ 2**: Đọc trạng thái nút nhấn:
```c
// Cấu hình
void GPIO_Init(){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Điều khiển
void Control(){
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0){
		while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0);
		if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)){
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		} else {
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
		}
	}

}

```

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


 <details>
	<summary><strong>BÀI 4: CÁC CHUẨN GIAO TIẾP CƠ BẢN</strong></summary>

## Bài 4: Các chuẩn giao tiếp cơ bản
### 1. Sơ lược về vấn đề truyển nhận dữ liệu:
- Truyền nhận dữ liệu trong vi điều khiển (MCU) là quá trình trao đổi tín hiệu điện áp giữa các chân (pin) của MCU.
- Do đó khi MCU A muốn truyền dữ liệu cho 1 MCU B, dữ liệu sẽ được đổi thành các tín hiệu điện áp tương ứng trên các chân mà 2 MCU giao tiếp.
![image](https://github.com/user-attachments/assets/d3fd3596-88b3-4c8b-b55a-f2934eb0b8ba)

### 2. SPI
- SPI (Serial Peripheral Interface) hay còn gọi là giao diện ngoại vi nối tiếp, được phát triển bởi hãng Motorola.
- Hoạt động ở chế độ song công toàn phần, có thể truyền và nhận ở cùng 1 thời điểm.
- Là giao tiếp đồng bộ nối tiếp, quá trình truyền nhận đều được đồng bộ với xung clock sinh bởi Master.
- Một Master có thể giao tiếp được nhiều Slave.
- Sử dụng 4 dây để giao tiếp:
  + **SCK** (Serial clock): xung clock tạo bởi Master cung cấp cho slave.
  + **MISO** (Master in - Slave out): Tín hiệu tạo bởi thiết bị Slave và nhận bởi thiết bị Master.
  + **MOSI** (Master out - Slave in): Tín hiệu tạo bởi thiết bị Master và nhận bởi thiết bị Slave.
  + **SS** (Slave Select) / **CS** (Chip Select): Chọn thiết bị slave cụ thể để giao tiếp. Để chọn slave giao tiếp với Master cần chủ động kéo đường SS tương ứng xuống mức thấp (0/low)
![image](https://github.com/user-attachments/assets/7a0c5de3-4c3d-44be-8560-d2882bdbeaf7)

- Quá trình truyền dữ liệu:
  + Master kéo chân CS của slave muốn truyền xuongs 0 để báo hiệu bắt đầu truyền nhận.
  + Master sẽ cung cấp xung clock. Với mối xung clock, 1 bit sẽ được truyền bởi Master và 1 bit sẽ được truyền bởi slave.
  + Các thanh ghi cập nhật lại giá trị và dịch trái 1 bit.
  + Lặp lại quá trình trên cho đến khi truyền đủ 8 bit trong thanh ghi.
![image](https://github.com/user-attachments/assets/f12b6222-9b6d-49fa-bb04-81ebbf8b719d)

- Có tất cả 4 chế độ hoạt động phụ thuộc vào 2 tham số CPOL (Clock Polarity) và CPHA (Clock Phase).
  + CPOL: bằng 0 --> Xung clock ban đầu ở mức 0, bằng 1 --> Xung clock ban đầu ở mức 1.
  + CPHA: bằng 0 --> Đọc dữ liệu ở cạnh thứ nhất, truyền dữ liệu ở cạnh thứ 2; bằng 1 --> Đọc dữ liệu ở cạnh thứ hai, truyền dữ liệu ở cạnh thứ nhất.

| SPI Mode | CPOL | CPHA | Hoạt động|
| :---: | :---: | :---: | :---: |
| 1 | 0 | 0 | xung nhịp ở mức thấp và dữ liệu được lấy mẫu khi cạnh lên (mặc định) |
| 2 | 0 | 1 | xung nhịp ở mức thấp và dữ liệu được lấy mẫu khi cạnh xuống |
| 3 | 1 | 0 | xung nhịp ở mức cao và dữ liệu được lấy mẫu khi cạnh lên |
| 4 | 1 | 1 | xung nhịp ở mức cao và dữ liệu được lấy mẫu khi cạnh xuống |

![image](https://github.com/user-attachments/assets/e677dc34-4e27-41f9-ad10-27c2ec02f73c)

- Ưu điểm và nhược điểm:
  + **Ưu điểm**: cho phép truyền dữ liệu với tốc độ rất nhanh, thường đạt được tốc độ Mbps hoặc thậm chí hàng chục Mbps; quá trình truyền ít bị lỗi do đồng bộ xung clock giữa Master và Slave; Có thể giao tiếp với nhiều Slave cùng lúc và giao tiếp song công (truyền nhận đồng thời).
  + **Nhược điểm**: Cần nhiều kết nối dây (4 dây), tốn tài nguyên phần cứng khi muốn giao tiếp với nhiều slave; Khoảng cách truyền ngắn.

### 3. UART
- UART (Universal Asynchronous Receiver-Transmitter) là một giao thức truyền thông phần cứng dùng giao tiếp nối tiếp không đồng bộ và có thể cấu hình được tốc độ.
- Là chuẩn giao tiếp nối tiếp, chỉ có 2 thiết bị giao tiếp với nhau.
- Sử dụng 2 dây giao tiếp là **Tx** (Truyền) và **Rx** (Nhận).
![image](https://github.com/user-attachments/assets/573b9eb6-2253-48f5-b9d5-9303a77aa063)

- Tốc độ truyền: được đặt ở 1 số chuẩn, gọi là Baudrate = Số bit truyền / 1s, đồng bộ giữa Slave và Master (Ví dụ: 9600, 19200,38400,... Các tốc độ khác nhau tùy thuộc vào ứng dụng hệ thống sử dụng).
- Có 3 chế độ truyền:
  + Simplex: Chỉ tiến hành giao tiếp một chiều.
  + Half duplex: Dữ liệu sẽ đi theo một hướng tại 1 thời điểm.
  + Full duplex: Thực hiện giao tiếp đồng thời đến và đi từ mỗi master và slave.
- Quá trình truyền nhận dữ liệu: Dữ liệu được truyền sẽ đóng thành các gói (packet), bao gồm
  + Start: 1 bit bắt đâu.
  + Bit dữ liệu: 5 - 9 bit.
  + Parity Bit (Bit chẵn lẽ): để kiểm tra lỗi bit khi truyền, gồm 2 quy luật
    * Quy luật chẵn: Thêm một bit '0' hoặc '1' để số bit '1' là số chẵn.
    * Quy luật lẻ: Thêm một bit '0' hoặc '1' để số bit '1' là số lẻ.

![image](https://github.com/user-attachments/assets/a682257e-88f0-4604-889a-8524a3210be5)

- **Ưu điểm và nhược điểm**:
  + **Ưu điểm**:
    * Đơn giản phổ biến.
    * Tốc độ có thể điều chỉnh linh hoạt.
    * Tiết kiệm phần cứng (chỉ dùng 2 dây để giao tiếp).
  + **Nhược điểm**:
    * Tốc độ truyền thấp hơn so với SPI.
    * Chỉ hỗ trợ giao tiếp đơn Master, đơn Slave.
    * Chỉ kiểm tra được số lẻ bit lỗi.

### 4. I2C
- Là chuẩn giao tiếp đồng bộ, nối tiếp (dữ liệu truyền từng bit theo 1 đường SDA duy nhất).
- Hoạt động ở chế độ bán song công (half duplex) vì tại 1 thời điểm chỉ có thể nhận hoặc truyền dữ liệu.
- Một Master có thể giao tiếp với nhiều Slave hoặc nhiều Master giao tiếp với 1 Slave.
- Sử dụng 2 dây để giao tiếp:
  + **SCL** (Serial Clock): Tín hiệu xung nhịp đồng bộ giữa Master và Slave được Master tạo ra.
  + **SDA** (Serial Data): Đường truyền và nhận dữ liệu giữa Master và Slave.

- Các bước truyền/nhận dữ liệu: dữ liệu trong I2C được truyền trong các tin nhắn được chia thành các khung dữ liệu như hình dưới đây:
![image](https://github.com/user-attachments/assets/aa2aa8e9-901d-48b8-8ac9-90ca890b5152)

  + **B1**: Master gửi điều kiện khởi động (Start) là chân SDA xuống mức 0 trước chân SCL.
  + **B2**: Master gửi 7 hoặc 10 bit địa chỉ để tìm Slave mà nó muốn giao tiếp.
  + **B3**: Bit R/W được gửi đi nếu bằng '0' khi Master muốn gửi dữ liệu đến Slave, '1' nếu muốn đọc dữ liệu từ Slave.
  + **B4**: Nếu địa chỉ được gửi đi trùng với địa chỉ của 1 Slave nào đó thì Slave đó sẽ gửi ACK (bit '0'), nếu không có Slave nào nhận thì sẽ giữ nguyên NACK (bit '1').
  + **B5**: Sau khi chọn được Slave để giao tiếp, Master sẽ đọc/gửi dữ liệu lần lượt một khung 8 bit từ/đến Slave, sau mỗi khung sẽ có một bit ACK được Slave phản hồi về cho Master (nếu ghi dữ liệu), hoặc Master gửi cho Slave (khi đọc dữ liệu) để xác nhận đã gửi/nhận thành công hay không.
  + **B6**: Gửi điều khiện Stop để kết thúc truyền nhận dữ liệu: chân SDA lên mức 1 trước chân SCL (Lưu ý: khi nhận được tín hiệu NACK thì bất cứ lúc nào cũng có thể nhảy đến bước 6 để kết thúc truyền nhận dữ liệu).
![image](https://github.com/user-attachments/assets/8bc49ba9-daf8-41f3-b4bb-5d08d5ffdbab)

- **Lưu ý khi dùng nhiều Master giao tiếp với 1 Slave**: có thể xảy ra sự cố khi 2 hay nhiều Master cùng gửi/nhận dữ liệu cùng lúc qua đường SDA. Lúc đó cần phát hiện xem đường SDA cao hay thấp trước khi truyền tin nhắn. Nếu SDA cao thì có thể truyền tin nhắn an toàn, ngược lại thì có 1 Master khác đang có quyền điều khiển bus nên các Master còn lại phải chờ.

- **Ưu điểm và nhược điểm**:
  + **Ưu điểm**:
    * Tiết kiệm phần cứng (2 dây).
    * Hỗ trợ giao tiếp với nhiều Slave hoặc nhiều Master.
  + **Nhược điểm**:
    * Tốc độ truyền thấp.
    * Quản lý địa chỉ phức tạp.
    * Khoảng cách truyền ngắn.

  </details>

 <details>
	<summary><strong>BÀI 5: GIAO TIẾP SPI</strong></summary>

## Bài 5: Giao tiếp SPI
### 1. SPI Software:
- Là 1 cách thức mô phỏng hoạt động của giao thức truyền thông SPI sử dụng GPIO của vi điều khiển.
- Các bước cấu hình mô phỏng:
  + B1: Xác định các chân GPIO.
  + B2: Cấu hình GPIO.
  + B3: Khởi tạo các chân cho SPI

#### a. Xác định các chân GPIO:
Giao tiếp SPI có 4 chân cơ bản:
- SCK: Xung clock đồng bộ được tạo bởi Master để đồng bộ tín hiệu truyền nhận dữ liệu giữa Master và Slave.
- MOSI (Master out Slave in): Tín hiệu để Master truyền dữ liệu cho Slave.
- MISO (Master in Slave out): nhận dữ liệu từ Slave truyền cho Master.
- CS (Chip select): Chọn thiết bị Slave cụ thể để giao tiếp. Để chọn Slave để giao tiếp, Master chủ động kéo đường dây tín hiệu xuống mức 0.
- Ta định nghĩa các chân trên ứng với các GPIO sau: 
```c
#define SPI_SCK_Pin GPIO_Pin_0
#define SPI_MISO_Pin GPIO_Pin_1
#define SPI_MOSI_Pin GPIO_Pin_2
#define SPI_CS_Pin GPIO_Pin_3
#define SPI_GPIO GPIOA
#define SPI_RCC RCC_APB2Periph_GPIOA
```

#### b. Cấu hình GPIO:
Cấp xung cho các GPIO và TIM2 để tạo hàm delay:
```c
void RCC_Config(){
	RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
```

Đối với Master:
```c
void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin | SPI_MOSI_Pin | SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
}

```
Đối với Slave
```c

void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin | SPI_MOSI_Pin | SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
}
```

Tạo xung clock:
```c
void Clock(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_SET);
	delay_ms(4);
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	delay_ms(4);
}
```
#### c. Khởi tạo các chân cho SPI:
```c
void SPI_Init(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
}
```

#### d. Hàm truyền và nhận dữ liệu:
- Hàm truyền: truyền lần lượt 8 bit trong byte dữ liệu, quá trình truyền như sau:
  + Kéo CS xuống 0:
    * Truyền 1 bit.
    * Dịch 1 bit.
    * Gửi clock().
  + Kéo CS lên 1.
```c
void SPI_Master_Transmit(uint8_t u8Data){	//0b10010000
	uint8_t u8Mask = 0x80;	// 0b10000000
	uint8_t tempData;
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_RESET);
	delay_ms(1);
	for(int i = 0; i < 8; i++){
		tempData = u8Data & u8Mask;
		if(tempData){
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_SET);
			delay_ms(1);
		} else{
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
			delay_ms(1);
		}
		u8Data = u8Data << 1;
		Clock();
	}
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	delay_ms(1);
}
```



- Hàm nhận: Nhận 8 bit dữ liệu theo các bước sau:
  + Kiểm tra CS == 0 để nhận biết bắt đầu quá trình giao tiếp.
    * Kiểm tra clock == 1 để bắt đầu nhận dữ liệu.
    * Đọc Data trên chân MOSI, đồng thời lưu lại vào 1 biến.
    * Dịch sang trái 1 bit.
    * Chờ cho clock == 1 lần nữa, lặp lại các bước trên.
  + Kiểm tra CS == 1 để kết thúc quá trình giao tiếp.

```c
uint8_t SPI_Slave_Receive(void){
	uint8_t dataReceive = 0x00;	//0b0000 0000
	uint8_t temp = 0x00;
	while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
	for(int i = 0; i < 8; i++){ 
		if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
			while (GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
				temp = GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin);
			}
			dataReceive <<= 1;
			dataReceive |= temp;
    		}
		while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
	}
	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
	return dataReceive;
}
```
### 2. SPI Hardware:
Trên mỗi vi điều khiển đều tích hợp modun giao tiếp SPI, được điều khiển bởi các thanh ghi, phần cứng GPIO khác nhau gọi là SPI cứng. STM32F1 có 2 khối SPI được tích hợp là SPI1 ở APB2 và SPI2 ở PAB1. Các khối này đều được xây dựng các kết nối, driver và các hàm riêng trong bộ thư viện chuẩn.
#### 1. Cấu hình GPIO cho SPI
STM32F1 đã cấu hình sẵn các chân phục vụ cho giao tiếp SPI, ta chỉ cần định nghĩa đúng với chức năng của chúng. Ở đây ta sử dụng SPI1, định nghĩa như sau:
```c
#define SPI1_NSS 	GPIO_Pin_4
#define SPI1_SCK	GPIO_Pin_5
#define SPI1_MISO 	GPIO_Pin_6
#define SPI1_MOSI 	GPIO_Pin_7
#define SPI1_GPIO 	GPIOA
```

Cấu hình GPIO:
```c
void GPIO_Cofig(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = SPI1_NSS| SPI1_SCK| SPI1_MISO| SPI1_MOSI; // Tất cả các chân cần cấu hình
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Ta thiết lập chế độ Alternate Function Push-Pull (cấu hình các chân hoạt động trong 1 chức năng thay thế vầ sử dụng chế độ push-pull).
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
}
```

#### 2. Cấu hình SPI
Tương tự các ngoại vi khác, các tham số SPI được cấu hình trong struct SPI_InitTypedef:
- `SPI_Mode`: Quy định chế độ hoạt động của thiết bị SPI. 
- `SPI_Direction`: Quy định kiểu truyền của thiết bị.
- `SPI_BaudRatePrescaler`: Hệ số chia clock cấp cho Module SPI.
- `SPI_CPOL`: Cấu hình cực tính của SCK . Có 2 chế độ:
  + `SPI_CPOL_Low`: Cực tính mức 0 khi SCK không truyền xung.
  + `SPI_CPOL_High`: Cực tính mức 1 khi SCK không truyền xung.
- `SPI_CPHA`: Cấu hình chế độ hoạt động của SCK. Có 2 chế độ:
  + `SPI_CPHA_1Edge`: Tín hiệu truyền đi ở cạnh xung đầu tiên.
  + `SPI_CPHA_2Edge`: Tín hiệu truyền đi ở cạnh xung thứ hai.
- `SPI_DataSize`: Cấu hình số bit truyền. 8 hoặc 16 bit.
- `SPI_FirstBit`: Cấu hình chiều truyền của các bit là MSB hay LSB.
- `SPI_CRCPolynomial`: Cấu hình số bit CheckSum cho SPI.
- `SPI_NSS`: Cấu hình chân SS là điều khiển bằng thiết bị hay phần mềm.

Hàm cấu hình tham số SPI:

- Cấu hình Master:
```c
void SPI_Config(){
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; // Cấu hình cho Master
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // Chế độ xong công
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // chia tầng số 72Mhz/16
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; // Cực tính mức 0 khi SCK không truyền xung
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; // Tín hiệu truyền ở cạnh xung đầu tiên
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // Kích thước Data = 8 bit
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB; // Truyền Data từ trái qua phải
	SPI_InitStructure.SPI_CRCPolynomial = 7; // 7 bit checksum
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // Điều khiển chân CS bằng phần mềm.
	
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
}
```
- Cấu hình Slave:
```c
void SPI_Config(){
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // chia tầng số 72Mhz/16
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; // Tín hiệu truyền ở cạnh xung thứ 2 để tránh xung đột vì Master truyền ở cạnh xung đầu tiên.
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; // Cực tính mức 0 khi SCK không truyền xung
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // Nhận 8 bit
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // Song công
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // Truyền Data từ phải qua trái
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave; // Cấu hình Slave
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // Điều khiển chân CS bằng phần mềm.
	
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
}
```

#### 3. Các hàm thông dụng và Hàm truyền nhận dữ liệu:
- Hàm SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data), tùy vào cấu hình datasize là 8 hay 16 bit sẽ truyền đi 8 hoặc 16 bit dữ liệu. Hàm nhận 2 tham số là bộ SPI sử dụng và data cần truyền.
- Hàm SPI_I2S_ReceiveData(SPI_TypeDef* SPIx) trả về giá trị đọc được trên SPIx. Hàm trả về 8 hoặc 16 bit data.
- Hàm SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG) trả về giá trị 1 cờ trong thanh ghi của SPI. Các cờ thường được dùng:
  + SPI_I2S_FLAG_TXE: Cờ báo truyền, cờ này sẽ set lên 1 khi truyền xong data trong buffer.
  + SPI_I2S_FLAG_RXNE: Cờ báo nhận, cờ này set lên 1 khi nhận xong data.
  + SPI_I2S_FLAG_BSY: Cờ báo bận,set lên 1 khi SPI đang bận truyền nhận.

Các hàm truyền nhận có thể viết như sau:

**Lưu ý**: Vì cấu hình NSS soft nên khi truyền, ta phải chủ động kéo SS xuống Low bằng phần mềm:

- Hàm truyền:
```c
void SPI_Send1Byte(uint8_t data){
    GPIO_WriteBit(SPI1_GPIO, SPI1_NSS, Bit_RESET); // Kéo chân CS xuống 0, bắt đầu quá trình truyền
   
    SPI_I2S_SendData(SPI1, data); // Truyền data thông qua bộ SPI1
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==0); // Chờ đến khi Data trong buffer truyền xong, cờ SPI_I2S_FLAG_TXE sẽ bằng 1
   
    GPIO_WriteBit(SPI1_GPIO, SPI1_NSS, Bit_SET); // Kéo chân CS lên 1, kết thúc quá trình truyền
}
```

- Hàm nhận:
```c
uint8_t SPI_Receive1Byte(void){
    uint8_t temp;
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==1);  // Chờ đến khi bộ SPI1 rảnh, khi cờ SPI_I2S_FLAG_BSY bằng 0
    temp = (uint8_t)SPI_I2S_ReceiveData(SPI1); // Tiến hành đọc data nhận được từ bộ SPI1 và lưu vào biến temp
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==0); // Chờ đến khi nhận xong data, khi đó cờ SPI_I2S_FLAG_RXNE = 1.
    return temp; // trả về data nhận được.
}

```

  </details>

 <details>
	<summary><strong>BÀI 6: GIAO TIẾP I2C</strong></summary>

## Bài 6: Giao tiếp I2C
### 1. I2C Software
#### a. Cấu hình GPIO cho I2C
Giao tiếp I2C sử dụng 2 chân truyền dữ liệu giữa các thiết bị:
- SDA: đường tín hiệu dữ liệu để truyền và nhận dữ liệu cho Master và Slave
- SCL: Đường tín hiệu mang xung nhịp clock để đồng bộ giữa Master và Slave

Giao thức I2C giao tiếp bán song công với khả năng giao tiếp nhiều Master và nhiều Slave.
![image](https://github.com/user-attachments/assets/0597b56f-2f90-4e51-994b-a57df2d5f389)

Định nghĩa và cấu hình các GPIO:
```c
// Định nghĩa các chân giao tiếp
#define I2C_SCL 	GPIO_Pin_6
#define I2C_SDA		GPIO_Pin_7
#define I2C_GPIO 	GPIOB

// Hàm cấp xung hoạt động cho timer và GPIO
void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

// Hàm cấu hình GPIO
void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = I2C_SDA| I2C_SCL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

```
#### b. Cấu hình I2C
![image](https://github.com/user-attachments/assets/db322a52-3f6c-4cd1-b4b1-dcc8dbddb303)

- Hàm khởi tạo I2C: các chân SDA và SCL đều được thiết lập ở mức cao
```c
void I2C_Config(){
	WRITE_SDA_1;
	delay_us(1);
	WRITE_SCL_1;
	delay_us(1);
}

```

- Các Macro được thiết lập sẵn cho việc điều khiển mức điện áp và đọc tín hiệu trên các chân SDA và SCL:
```c
#define WRITE_SDA_0 	GPIO_ResetBits(I2C_GPIO, I2C_SDA)
#define WRITE_SDA_1 	GPIO_SetBits(I2C_GPIO, I2C_SDA)
#define WRITE_SCL_0 	GPIO_ResetBits(I2C_GPIO, I2C_SCL)
#define WRITE_SCL_1 	GPIO_SetBits(I2C_GPIO, I2C_SCL)
#define READ_SDA_VAL 	GPIO_ReadInputDataBit(I2C_GPIO, I2C_SDA)
```

- Tín hiệu Start: SDA kéo xuống mức 0 trước SCL 1 khoảng delay nhỏ
```c
void I2C_Start(){
	
	WRITE_SCL_1;  	
	delay_us(3);	
	WRITE_SDA_1;
	delay_us(3);
	WRITE_SDA_0;	//SDA reset to 0 before SCL.
	delay_us(3);
	WRITE_SCL_0;
	delay_us(3);
}

```
- Tín hiệu Stop: SCL kéo lên trước SDA 1 khoảng delay nhỏ
```c
void I2C_Stop(){
	
	WRITE_SDA_0;
	delay_us(3);
	WRITE_SCL_1; 	//SCL set to 1 before SDA.
	delay_us(3);
	WRITE_SDA_1;
	delay_us(3);
}

```

#### c. Hàm truyền và hàm nhận
- Qúa trình truyền nhận dữ liệu: Master truyền lần lượt 7/10 bit địa chỉ + 1 bit R/W trên đường SDA để chỉ định địa chỉ slave cần giao tiếp, đồng thời nhận lại ACK từ Slave xác nhận tồn tại Slave giao tiếp. Sau đó Master mới tổ chức truyền từng 8 bit dữ liệu đến cho Slave, đồng thời nhận lại phản hồi ACK tương ứng từ Slave.

- Hàm truyền: truyền lần lượt 8 bitr trong byte dữ liệu:
  + Truyền 1 bit.
  + Tạo 1 clock.
  + Dịch 1 bit.
  
  Chờ nhận ACK ở xung thứ 9.
```c
status I2C_Write(uint8_t u8Data){	
	uint8_t i;
	status stRet;
	for(int i=0; i< 8; i++){	//Write byte data.
		if (u8Data & 0x80) {
			WRITE_SDA_1;
		} else {
			WRITE_SDA_0;
		}
		delay_us(3);
		WRITE_SCL_1;
		delay_us(5);
		WRITE_SCL_0;
		delay_us(2);
		u8Data <<= 1;
	}
	WRITE_SDA_1;					
	delay_us(3);
	WRITE_SCL_1;		
	delay_us(3);
	
	if (READ_SDA_VAL) {	
		stRet = NOT_OK;				
	} else {
		stRet = OK;					
	}
	delay_us(2);
	WRITE_SCL_0;
	delay_us(5);
	
	return stRet;
}
```

- Hàm nhận: Nhận lần lượt 8 bit dữ liệu trên đường SDA
  + Kéo SDA lên 1 để đọc dữ liệu:
    * Đọc Data trên SDA, ghi vào biến.
    * Dịch 1 bit.
  + Gửi 1 bit ACK phản hồi về cho Master ở xung thứ 9.
```c
uint8_t I2C_Read(ACK_Bit _ACK){	
	uint8_t i;						
	uint8_t u8Ret = 0x00;
	WRITE_SDA_1;
	delay_us(3);	
	for (i = 0; i < 8; ++i) {
		u8Ret <<= 1;
		WRITE_SCL_1;
		delay_us(3);
		if (READ_SDA_VAL) {
			u8Ret |= 0x01;
		}
		delay_us(2);
		WRITE_SCL_0;
		delay_us(5);
	}
	if (_ACK) {	
		WRITE_SDA_0;
	} else {
		WRITE_SDA_1;
	}
	delay_us(3);
	
	WRITE_SCL_1;
	delay_us(5);
	WRITE_SCL_0;
	delay_us(5);
	return u8Ret;
}
```

### 2. I2C Hardware
STM32F1 có 2 khối I2C: I2C1 và I2C2 ở APB1.
#### a. Cấu hình GPIO cho I2C
Ta sử dụng bộ I2C1, là bộ I2C được cấu hình sẵn, ta cần định nghĩa các chân:
```c
#define I2C_SCL 	GPIO_Pin_6
#define I2C_SDA		GPIO_Pin_7

#define I2C1_GPIO 	GPIOB
```
- SDA: Input/Output
- SCL: Output
- Vì có trở kéo lên nên hoạt động ở chế độ OD

![image](https://github.com/user-attachments/assets/fdf40952-085b-4b67-884d-6c840f1588b6)

Cấu hình GPIO:
```c
void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
```

#### b. Cấu hình I2C

Tương tự các ngoại vi khác, I2C cũng được cấu hình bằng Struct I2C_InitTypeDef:

- `I2C_Mode`: Cấu hình chế độ hoạt động cho I2C
  + `I2C_Mode_I2C`: Chế độ I2C FM (Fast Mode).
  + `I2C_Mode_SMBusDevice` & `I2C_SMBusHost`: Chế độ SM (Slow Mode).
- `I2C_ClockSpeed`: Cấu hình clock cho I2C, tối đa 100khz với SM và 400khz ở FM.
- `I2C_DutyCycle`: Cấu hình chu kì nhiệm vụ của xung:
  + `I2C_DutyCycle_2`: Thời gian xung thấp/ xung cao =2;
  + `I2C_DutyCycle_16_9`: Thời gian xung thấp/ xung cao =16/9;
- `I2C_OwnAddress1`: Cấu hình địa chỉ slave.
- `I2C_Ack`: Cấu hình ACK, có sử dụng ACK hay không.
- `I2C_AcknowledgedAddress`: Cấu hình số bit địa chỉ. 7 hoặc 10 bit.

Cấu hình I2C:
```c
void I2C_Config()
{
	
	I2C_InitTypeDef I2C_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); // Cấp clock cho I2C1
	I2C_InitStructure.I2C_ClockSpeed = 400000; // Cấu hình clock 400 kHz
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; // Chế độ Fast Mode
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; // Tỷ lệ xung thấp/xung cao = 2
	I2C_InitStructure.I2C_OwnAddress1 = 0x33; // Địa chỉ Slave
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; // Sử dụng ACK
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // Sử dụng 7 bit địa chỉ

	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}
```

#### c. Các hàm thông dụng
- Hàm I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction), gửi đi 7 bit address để xác định slave cần giao tiếp. Hướng truyền được xác định bởi I2C_Direction để thêm bit RW.
- Hàm I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data) gửi đi 8 bit data.
- Hàm I2C_ReceiveData(I2C_TypeDef* I2Cx) trả về 8 bit data.
- Hàm I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT) trả về kết quả kiểm tra I2C_EVENT tương ứng:
- Hàm I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT) trả về kết quả kiểm tra I2C_EVENT tương ứng:
  + I2C_EVENT_MASTER_MODE_SELECT: Đợi Bus I2C về chế độ rảnh.
  + I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: Đợi xác nhận của Slave với yêu cầu ghi của Master.
  + I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED: Đợi xác nhận của Slave với yêu cầu đọc của Master.
  + I2C_EVENT_MASTER_BYTE_TRANSMITTED: Đợi truyền xong 1 byte data từ Master.
  + I2C_EVENT_MASTER_BYTE_RECEIVED: Đợi Master nhận đủ 1 byte data.

#### d. Hàm truyền và hàm nhận
Qúa trình truyền nhận:
- Bắt đầu truyền nhận, bộ I2C sẽ tạo 1 tín hiệu start. Đợi tín hiệu báo Bus sẵn sàng.
- Gửi 7 bit địa chỉ để xác định slave. Đợi Slave xác nhân.
- Gửi/đọc các byte data. Đợi truyền xong.
- Sau đó kết thúc bằng tín hiệu Stop



- Qúa trinh Start và gửi 7 bit:

```c
I2C_GenerateSTART(I2C1, ENABLE);
 //Waiting for flag
 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
I2C_Send7bitAddress(I2C1, 0x44, I2C_Direction_Transmitter);
//And check the transmitting
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
```

- Hàm truyền:
```c
void Send_I2C_Data(uint8_t data)
{
	I2C_SendData(I2C1, data);
	// wait for the data trasnmitted flag
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
```

- Hàm nhận:
```c
uint8_t Read_I2C_Data(){
	
	uint8_t data = I2C_ReceiveData(I2C1);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	return data;
}

```

- Tín hiệu Stop:
```c
I2C_GenerateSTOP(I2C1, ENABLE);
```
  </details>

 <details>
	<summary><strong>BÀI 7: GIAO TIẾP UART</strong></summary>

## Bài 7: Giao tiếp UART
### 1. UART Software
#### a. Cấu hình GPIO cho UART Software
UART sử dụng 2 chân để giao tiếp, đó là Tx(Transmit) và Rx(Receive).
![image](https://github.com/user-attachments/assets/3329f813-562c-48c0-95f8-9982fac8e369)

Định nghĩa các chân giao tiếp:
```c
#define TX_Pin		GPIO_Pin_9
#define RX_Pin		GPIO_Pin_10
#define UART_GPIO 	GPIOA

```

Cấu hình GPIO cho UART: Tx là chân truyền nên được cấu hình OUTPUT, còn Rx là chân nhận nên cấu hình INPUT.
```c
void GPIO_Config(){
	// Cấp clock cho GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIOInitStruct;
	// Cấu hình chân Rx là Input Floating
	GPIOInitStruct.GPIO_Pin = RX_Pin;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART_GPIO, &GPIOInitStruct);

	// Cấu hình chân Tx là Output PushPull
	GPIOInitStruct.GPIO_Pin = TX_Pin;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(UART_GPIO, &GPIOInitStruct);
}

```

#### b. Thiết lập Baurate
Tốc độ Baurate được xác định bởi thời gian truyền đi 1 bit. Ta dùng tốc độ phổ thông là 9600, tương ứng thời gian truyền mỗi bit là 105us

Định nghĩa thời gian truyền dữ liệu
```c 
#define BRateTime 105
```

#### c. Cấu hình Uart
Ở chế độ nghỉ (không truyền), đường Tx được giữ ở mức cao. 

Hàm UART_Config dùng để thiết lập chế độ nghỉ cho đường truyền:
```c
void UARTSoftware_Init(){
GPIO_SetBits(UART_GPIO, TX_Pin);
	delay_us(1);
}
```

#### d. Hàm truyền và hàm nhận
- Hàm truyền:
  ![image](https://github.com/user-attachments/assets/0d2c4600-fa92-4f3e-af0c-438014826714)

Hàm truyền truyền lần lượt 8 bit trong byte dữ liệu, sau khi tín hiệu start được gửi đi. Quy trình như sau:

  + Tạo tín hiệu START, delay 1 khoảng thời gian
    * Truyền mỗi bit dữ liệu, truyền mỗi bit trong 1 period time.
    * Dịch 1 bit.
  +  Tạo tín hiệu STOP, delay tương ứng với số bit stop.

```c
void UARTSoftware_Transmitt(const char DataValue) {
	// Start bit
	GPIO_ResetBits(GPIOA, TX_Pin);
	delay_us(BRateTime);
	
	// Truyền các bit dữ liệu (LSB trước)
	for (int i = 0; i < 8; i++) {
	if (DataValue & (1 << i)) {
	    GPIO_SetBits(GPIOA, TX_Pin);
	} else {
	    GPIO_ResetBits(GPIOA, TX_Pin);
	}
	delay_us(BRateTime);
	}
		
	// Stop bit
	GPIO_SetBits(GPIOA, TX_Pin);
	delay_us(BRateTime);
}
```

- Hàm nhận:
![image](https://github.com/user-attachments/assets/f43c0476-2cd6-4217-b271-a7308c3cd591)

Hàm nhận sẽ nhận lần lượt 8 bit:
  + Chờ tín hiệu START từ thiết bị gửi.
  + Delay 1.5 preriod time.
    * Đọc Data chân Rx và ghi vào biến.
    * Dịch 1 bit.
    * Delay 1 period time.
  + Delay 0.5 preriod time và chờ STOP bit.

```c
unsigned char UARTSoftware_Receive() {
    unsigned char DataValue = 0;

    // Ðợi Start bit
    while (GPIO_ReadInputDataBit(GPIOA, RX_Pin) == 1);

    // Đợt 1.5 period time
    delay_us(BRateTime + BRateTime / 2);

    // Ðọc lần lượt các bit dữ liệu trên chân Rx (LSB trước)
    for (int i = 0; i < 8; i++) {
				
        if (GPIO_ReadInputDataBit(GPIOA, RX_Pin)) {
            DataValue |= (1 << i);
        }
	delay_us(BRateTime); // Đợi 1 Period time
    }

    // Ðợi Stop bit
    delay_us(BRateTime / 2);

    return DataValue;
}

```

#### e. Parity bit
Là bit chẵn/lẻ được thêm vào cuối Data để kiểm tra dữ liệu truyền đi có bị lỗi bit hay không.

Cấu hình các chế độ gồm là bit chẵn/lẻ hay không dùng parity bit:
```c
typedef enum{
	Parity_Mode_NONE,
	Parity_Mode_ODD,
	Parity_Mode_EVENT
}Parity_Mode;

```

Tùy vào cấu hình parity là chẵn hay lẻ mà thiết bị truyền có thể thêm bit parity là 0 hoặc 1.

Phía nhận cấu hình parity giống như phía truyền, sau khi nhận đủ các bit sẽ kiểm tra parity có đúng hay không.

- Hàm tạo Parity bit
```c
uint16_t Parity_Generate(uint8_t data, Parity_Mode Mode){
	uint8_t count = 0;
	uint8_t data1 = data;
	for(int i = 0; i < 8; i++){
		if(data1 & 0x01){
			count++;
		}
		data1 >>= 1;
	}
	switch(Mode){
		case Parity_Mode_NONE:
			return data; 
			break;
		case Parity_Mode_ODD:
			if(count % 2){
				return ((data << 1) | 1);
			} else {
				return (data<<1);
			}
			break;
		case Parity_Mode_EVEN:
			if(!(count % 2)){
				return ((data << 1) | 1);
			} else {
				return (data << 1);
			}
			break;
		default:
			return data;
			break;
	}
}
```

- Hàm kiểm tra Parity:
```c
uint8_t Parity_Check(uint8_t data, Parity_Mode Mode){
	uint8_t count = 0;
	for(int i = 0; i < 8; i++){
		if(data & 0x01){
			count++;
		}
		data >>= 1;
	}
	switch(Mode){
		case Parity_Mode_NONE:
			return 1; 
			break;
		case Parity_Mode_ODD:
			return (count % 2);
			break;
		case Parity_Mode_EVEN:
			return (!(count % 2));
			break;
		default:
			return 0;
			break;
	}
}
```

### 2. UART Hardware

  </details>
