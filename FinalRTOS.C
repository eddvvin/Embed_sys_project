//Final RTOS of Embedded Systems.


typedef struct task
{
    int state;
    unsigned long elapsedTime;
    unsigned int taskNum;
    int (*Function) (int);
} task;

const unsigned int NumTasks = ##; //Total number of tasks in device...
const unsigned long period = ##; //Modify to run fast enough....
const unsigned long TaskPeriod[#] = {##,##,##}; //Number of tasks...
task tasks[#]; 

//statemachine code like enum stuff here
//plus refrence function links below...

void TimerISR(){
    unsigned char i;
    for(i = 0; i < NumTasks; i++){
        if (tasks[i].elapsedTime >= TaskPeriod[tasks[i].taskNum]){
            tasks[i].state = tasks[i].Function(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += period;
    }
}

int main(){
    tasks[0].state = ###; //Init state
    tasks[0].elapsedTime = TaskPeriod[0];
    tasks[0].taskNum = 0;
    tasks[0].Function = &#####; //insert name of statemachine



    TimerSet(period);
    TimerOn();

    while(1);
    return 0;
}


//task implimentatio here


