#include "bsp_can.h"
#include "main.h"
#include "bsp_headfile.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_measure_t Motor_measure[14];
Motor_measure_t test_motor;
static CAN_TxHeaderTypeDef CANx_tx_message;
static uint8_t CANx_send_data[8];

void can_filter_init(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef can_filter_st;                                                                            //����������ṹ��
    can_filter_st.FilterActivation = ENABLE;                                                            //ENABLEʹ�ܹ�����
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;                                            //���ù�����ģʽ--��ʶ������λģʽ
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;                                        //��������λ�� 32 λ
    can_filter_st.FilterIdHigh = 0x0000;                                                                    //ID��λ
    can_filter_st.FilterIdLow = 0x0000;                                                                        //ID��λ
    can_filter_st.FilterMaskIdHigh = 0x0000;                                                            //�����������λ
    can_filter_st.FilterMaskIdLow = 0x0000;                                                                //�����������λ
//    can_filter_st.FilterBank = 0;                                                                                    //��������-˫CAN��ָ��0~27
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                                        //������������� FIFO
    if (hcan->Instance == CAN1) {
        can_filter_st.FilterBank = 0;
    } else if (hcan->Instance == CAN2) {
        can_filter_st.FilterBank = 14;
    }
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);                                                    //HAL�����ù���������
    HAL_CAN_Start(hcan);                                                                                                //ʹ��CAN������
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);        //ʹ��CAN�ĸ����ж�
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

//    can_filter_st.SlaveStartFilterBank = 14;                                                            //˫CANģʽ�¹涨CAN������ģʽ�Ĺ��������䣬�ӹ�����Ϊ14
//    can_filter_st.FilterBank = 14;                                                                                //��������-˫CAN��ָ��0~27
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);                                                    //HAL�����ù���������
//    HAL_CAN_Start(&hcan2);                                                                                                //ʹ��CAN������
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);        //ʹ��CAN�ĸ����ж�
}

void Motor_measure_fun(Motor_measure_t *ptr, uint8_t *RX_buffer) {
    ptr->last_angle = ptr->angle;                                                                                        //��¼��һ��ת�ӻ�е�Ƕ�
    ptr->angle = (uint16_t) ((RX_buffer)[0] << 8 | (RX_buffer)[1]);                    //����ת�ӻ�е�Ƕ�
    ptr->speed = (uint16_t) ((RX_buffer)[2] << 8 | (RX_buffer)[3]);                    //����ת��ת��(rpm)
    ptr->torque_current = (uint16_t) ((RX_buffer)[4] << 8 | (RX_buffer)[5]);    //����ת�ص���
    ptr->temp = (RX_buffer)[6];

    if (ptr->angle - ptr->last_angle > 4096)                                                                    //����ת�ӱ��λ�е�Ƕ����ϴλ�е�ǶȵĲ�ֵ
        ptr->round_cnt--;                                                                                                        //�жϵ��ת������������
    else if (ptr->angle - ptr->last_angle < -4096)                                                        //�Ӷ�����õ�ת�Ӵ��ϵ翪ʼת������Ȧ��
        ptr->round_cnt++;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle;                                    //������Ȧ������ת���ܽǶ�
    //(�����ܽǶ�Ҫ���ݼ��ٱ��ٻ���
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RX_Header;                                                                                    //��������֡��֡ͷ
    uint8_t RX_BUFFER[8];                                                                                                        //���մ������֡���ݵ�����

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RX_Header, RX_BUFFER);                        //��can���յ�������֡����ֲ�����

    static uint8_t i = 0;
    if (hcan == &hcan1)
    {
//        usart_printf("%d\r\n",1);//can1
//        i = RX_Header.StdId -Chassis_3508A;//ͨ���������ݵ�IDȷ����һ�����ݴ�ŵĵ�ַ
//        Motor_measure_fun(&Motor_measure[i], RX_BUFFER);                                            //���ú��������ݴ���ṹ������

        Motor_measure_fun(&test_motor, RX_BUFFER);
    }
    else if (hcan == &hcan2)
    {                                                                                                    //can1
        i = RX_Header.StdId - CAN2_3508_ID1 +7;                                                            //ͨ���������ݵ�IDȷ����һ�����ݴ�ŵĵ�ַ
        Motor_measure_fun(&Motor_measure[i], RX_BUFFER);                                            //���ú��������ݴ���ṹ������
    }
}

void Set_motor_cmd(CAN_HandleTypeDef *hcan, uint32_t STDID,
                   int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    uint32_t send_mail_box;                                                                                                    //����һ���������ڴ洢����������
    CANx_tx_message.StdId = STDID;                                                                                    //��ʶ�����β����ݴ��뷢�͵����ݰ�
    CANx_tx_message.IDE = CAN_ID_STD;                                                                                //��ʶ��ѡ��λ��STD-��׼֡
    CANx_tx_message.RTR = CAN_RTR_DATA;                                                                            //����֡����
    CANx_tx_message.DLC = 0x08;                                                                                            //����֡����Ϊ8λ

    CANx_send_data[0] = motor1>> 8;                                                                     //���ν�Ҫ���͵����������������飬��ͬ
    CANx_send_data[1] = motor1;
    CANx_send_data[2] = motor2 >> 8;
    CANx_send_data[3] = motor2;
    CANx_send_data[4] = motor3 >> 8;
    CANx_send_data[5] = motor3;
    CANx_send_data[6] = motor4 >> 8;
    CANx_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(hcan,
                         &CANx_tx_message,                                                       //hal��can���ͺ������ú���������������
                         CANx_send_data, &send_mail_box);                                        //��ӷ��ͱ��ģ������������
}

void Set_motor1_cmd(CAN_HandleTypeDef *hcan, uint16_t motor_current)
{
    uint32_t send_mail_box;                                                                                                    //����һ���������ڴ洢����������
    CANx_tx_message.StdId = 0x200;                                                                                    //��ʶ�����β����ݴ��뷢�͵����ݰ�
    CANx_tx_message.IDE = CAN_ID_STD;                                                                                //��ʶ��ѡ��λ��STD-��׼֡
    CANx_tx_message.RTR = CAN_RTR_DATA;                                                                            //����֡����
    CANx_tx_message.DLC = 0x08;                                                                                            //����֡����Ϊ8λ
    CANx_send_data[0] = motor_current>> 8;                                                                     //���ν�Ҫ���͵����������������飬��ͬ
    CANx_send_data[1] = motor_current;

    HAL_CAN_AddTxMessage(hcan,
                         &CANx_tx_message,                                                       //hal��can���ͺ������ú���������������
                         CANx_send_data, &send_mail_box);                                        //��ӷ��ͱ��ģ������������
}



