/**
 * @file		FEdensity.h
 * @brief		FEdensity main header.
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 *
 * The interface for the FEdensity library.
 */

#ifndef FEDENSITY_H
#define FEDENSITY_H

#include "version.h"
#include <stdlib.h>

typedef struct {
    double x[3];
} Point;

struct node_;
typedef struct node_ {
    Point *x;
    struct node_ *next;
    struct node_ *prev;
} Node;

Node *listNew(const Point *x, int circular);
Node *listInsertAfter(Node *node, const Point *x);
Node *listNext(const Node *node);
Node *listDelete(Node *node);

#endif // FEDENSITY_H
