/**
 * @file		main.c
 * @brief		FEdensity main header.
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 *
 * Stand-alone FEdensity program.
 */

#include <stdio.h>
#include "FEdensity.h"

int main(){
    printf("FEdensity %s running\n",VERSION);

    Point x = {{0,1,2}};
    Point y = {{1,2,3}};
    Point z = {{2,3,4}};

    Node *list = listNew(&x,1);
    Node *l = list;
    list = listInsertAfter(list,&y);
    list = listInsertAfter(list,&z);

    int i = 0;
    while(i<10 && l){
        printf("data: %f\n",l->x->x[0]);
        i++;
        l = listNext(l);
    }

    l = listDelete(l);
    // while(i<10 && l){
        // printf("data: %f\n",l->x->x[0]);
        // i++;
        // l = listNext(l);
    // }

    return 0;
}
