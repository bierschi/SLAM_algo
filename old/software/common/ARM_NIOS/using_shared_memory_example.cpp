/*!
 * @file
 * */
/**
* @Author: florian
* @Date:   2017-03-12T12:31:18+01:00
* @Last modified by:   florian
* @Last modified time: 2017-03-12T12:31:18+01:00
*
* @brief This is a example how to use the Alf_SharedMemoryComm in a proper way. This example is not compileable!
*/

 Alf_SharedMemoryComm communication;

void this_is_my_interruptroutine(void *mess){
    uint32_t cs = alt_irq_disable_all();    //disables all other interrupts
    communication.ReadInterruptHandler();
    IORD(MAILBOX_BASE_REGISTER, 0);     // this can be done to guerentee a proper read on the register which leads to the interrupt!
    alt_irq_enable_all(cs);
}


int main(){

    communication.Init(...);

    alt_irq_register(INTERRUPT_NUMBER, 0, this_is_my_interruptroutine); //registing the interrupt

    while(1){...}

    return 0;
}
