/**
 * @file		FEdensity.h
 * @brief		FEdensity main header.
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 *
 * The implementation for the FEdensity library.
 */

#include "FEdensity.h"
#include <stdio.h>

Node *listNew(const Point *x, int circular){
    Node *node = (Node*)malloc(sizeof(node));
    node->x = (Point *)x;
    if(circular){
        node->next = node;
        node->prev = node;
    } else {
        node->next = NULL;
        node->prev = NULL;
    }
    return node;
}

Node *listInsertAfter(Node *node, const Point *x){
    Node *newNode = (Node*)malloc(sizeof(node));
    newNode->x = (Point *)x;
    newNode->next = node->next;
    node->next = newNode;
    return newNode;
}

Node *listNext(const Node *node){
    return node->next;
}

Node *listDelete(Node *node){
    Node *next = node->next;
    Node *prev = node->prev;
    printf("%x\n",next->next->next);
    // prev->next = next;
    // node->next->prev = node->prev;
    // free(node);
    return next;
}
