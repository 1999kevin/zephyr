
// #include "function.h"

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>

void PrintFloat(float value);
int get_ranking(float value);


/* 这个list是排位电压值的分隔值*/
float sort_list[20] = {0.137626031,
                       0.352843422,
                       0.512758375,
                       0.678310214,
                       0.843718871,
                       0.990500661,
                       1.119858713,
                       1.260164835,
                       1.411263736,
                       1.568406593,
                       1.725,
                       1.89,
                       2.050714286,
                       2.195089286,
                       2.338429054,
                       2.495358402,
                       2.651830664,
                       2.798987854,
                       2.950961538,
                       3.09076087
                    };  

static struct adc_channel_cfg adc_ch_cfg = {
	.gain  = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 2,
	.differential  = 0
};


uint16_t adc_buffer[10] = {0};

struct adc_sequence sequence = {
	.options = NULL,
	.channels = BIT(2),
	.buffer = adc_buffer,
	.buffer_size = 10,
	.resolution = 12
};

void main(void)
{
    // printk("ok to run\n");
    struct device *dev_GPIOA = device_get_binding("GPIOA");
    struct device *dev_GPIOB = device_get_binding("GPIOB");
    struct device *dev_ADC1 = device_get_binding("ADC_1");

    int ret;
	if (dev_GPIOB == NULL || dev_GPIOA == NULL ||dev_ADC1 == NULL) {
		printk("cannnot find GPIOA or GPIOB or ADC1\n");
		return;
	}

    int ret_PA12, ret_PA11, ret_PA15, ret_PB3, ret_PB4, ret_PB5, ret_PB6, ret_PB7;
    ret_PA12 = gpio_pin_get(dev_GPIOA, 12);
    ret_PA11 = gpio_pin_get(dev_GPIOA, 11);
    ret_PA15 = gpio_pin_get(dev_GPIOA, 15);
    ret_PB3 = gpio_pin_get(dev_GPIOB, 3);
    ret_PB4 = gpio_pin_get(dev_GPIOB, 4);
    ret_PB5 = gpio_pin_get(dev_GPIOB, 5);
    ret_PB6 = gpio_pin_get(dev_GPIOB, 6);
    ret_PB7 = gpio_pin_get(dev_GPIOB, 7);
    printk("PA12 PA11 PA15  PB3 PB4 PB5 PB6 PB7 = %d%d%d%d %d%d%d%d\n", 
            ret_PA12, ret_PA11, ret_PA15, ret_PB3, ret_PB4, ret_PB5, ret_PB6, ret_PB7);




    int size = 10;	
    int adc_rankings[size];
    float adc_results[size];
    int i;
    for(i=0; i<size; i++){
        adc_ch_cfg.channel_id = (uint8_t)(i);
        sequence.channels = BIT(i);
        ret = adc_channel_setup(dev_ADC1, &adc_ch_cfg);
	    if (ret != 0) {
	    	printk("Setting up of adc channel failed with code %d\n", ret);
		    return;
        }
        if(adc_read(dev_ADC1,&sequence) < 0){
		    printk("cannot read value\n");
            return;
	    }else{
            adc_results[i] = adc_buffer[0]/4095.0*3.3;
            adc_rankings[i] = get_ranking(adc_results[i]);
        }
    }

    for(i=0; i<size; i++){
        printk("AD%d, ",i);
        PrintFloat(adc_results[i]);
        printk("V, %d \n",adc_rankings[i]);
    }

}


/* get_ranking()
 * this function would get ranking of a voltage value
 */
int get_ranking(float value){
    for(int i=0;i<20;i++){
        if(value<sort_list[i]){
            return i+1;
        }
    }
    return 21;
}

/* PrintFloat()
 * This function helps to printk floating point in console 
 */ 
void PrintFloat(float value){
	int tmp,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6;
	tmp = (int)value;
	tmp1=(int)((value-tmp) * 10) % 10;
	tmp2=(int)((value-tmp) * 100) % 10;
	tmp3=(int)((value-tmp) * 1000) % 10;
	tmp4=(int)((value-tmp) * 10000) %10;
	tmp5=(int)((value-tmp) * 100000) %10;
	tmp6=(int)((value-tmp) * 1000000) %10;
	printk("%d.%d%d%d%d%d%d ",tmp,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6);

}


