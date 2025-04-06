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

- Ưu điểm và nhược điểm:
  + Ưu điểm: cho phép truyền dữ liệu với tốc độ rất nhanh, thường đạt được tốc độ Mbps hoặc thậm chí hàng chục Mbps; quá trình truyền ít bị lỗi do đồng bộ xung clock giữa Master và Slave; Có thể giao tiếp với nhiều Slave cùng lúc và giao tiếp song công (truyền nhận đồng thời).
  + Nhược điểm: Cần nhiều kết nối dây (4 dây), tốn tài nguyên phần cứng khi muốn giao tiếp với nhiều slave; Khoảng cách truyền ngắn.

### 3. UART
### 4. I2C

  </details>
