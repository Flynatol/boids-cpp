#include "taskmaster.h"
#include "tm_shared.h"

void function1(uint8_t thread_id) {
    //DEBUG("Function 1 executed on thread %d\n", thread_id);
}

void function2(uint8_t thread_id) {
   //DEBUG("Function 2 executed on thread %d\n", thread_id);
}


//////////////////////////////////////////////////////////
/// https://gist.github.com/Youka/4153f12cf2e17a77314c ///
//////////////////////////////////////////////////////////

/* Windows sleep in 100ns units */
BOOLEAN nanosleep(LONGLONG ns){
    /* Declarations */
    HANDLE timer;   /* Timer handle */
    LARGE_INTEGER li;   /* Time defintion */
    /* Create timer */
    if(!(timer = CreateWaitableTimer(NULL, TRUE, NULL)))
        return FALSE;
    /* Set timer properties */
    li.QuadPart = -ns;
    if(!SetWaitableTimer(timer, &li, 0, NULL, NULL, FALSE)){
        CloseHandle(timer);
        return FALSE;
    }
    /* Start & wait for timer */
    WaitForSingleObject(timer, INFINITE);
    /* Clean resources */
    CloseHandle(timer);
    /* Slept without problems */
    return TRUE;
}

