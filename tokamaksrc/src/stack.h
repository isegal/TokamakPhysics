/*************************************************************************
 *                                                                       *
 * Tokamak Physics Engine, Copyright (C) 2002-2007 David Lam.            *
 * All rights reserved.  Email: david@tokamakphysics.com                 *
 *                       Web: www.tokamakphysics.com                     *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

#ifndef NE_STACK_H
#define NE_STACK_H

#define NE_MAX_REST_ON 3

class neStackHeader;
class neRigidBody;

class neStackInfo {
public:
    void Init() {
        stackHeader = nullptr;
        body = nullptr;
        isTerminator = true;
        isBroken = false;
    }

    void Resolve();

    void AddToSolver(bool addCheader);

    neStackHeader *CheckAcceptNewHeader(neStackHeader *newHeader);

    void ForceAcceptNewHeader(neStackHeader *newHeader);

    //void Break();

    void CheckHeader(neStackHeader *sh);

    bool isResolved;
    bool isTerminator;
    bool isBroken;
    neStackHeader *stackHeader;
    neRigidBody *body;
    s32 startTime;
    s32 endTime;

    //neRestRecord restRecords[neRigidBody::NE_RB_MAX_RESTON_RECORDS];
};

using neStackInfoHeap = neDLinkList<neStackInfo>;

using neStackInfoItem = neFreeListItem<neStackInfo>;

class neStackHeader {
public:
    neSimulator *sim;

    neStackInfo *head;
    neStackInfo *tail;
    s32 infoCount;
    bool isHeaderX;
    bool isAllIdle;
    static s32 golbalTime;
    bool dynamicSolved;

    void Null() {
        head = nullptr;

        tail = nullptr;

        infoCount = 0;

        isHeaderX = false;

        isAllIdle = false;

        dynamicSolved = false;
    }

    //void Purge();

    void Resolve();

    void CheckLength() {
        s32 c = 0;

        neStackInfoItem *item = (neStackInfoItem *) head;

        while (item) {
            ASSERT(c < infoCount);

            c++;

            item = item->next;
        }
    }

    void CheckHeader() {
        ASSERT(infoCount != 0);

        s32 c = 0;

        neStackInfoItem *item = (neStackInfoItem *) head;

        while (item) {
            ASSERT(c < infoCount);

            c++;

            neStackInfo *sinfo = (neStackInfo *) item;

            ASSERT(sinfo->stackHeader == this);

            if (!sinfo->isTerminator)
                sinfo->CheckHeader(this);

            item = item->next;
        }
        ASSERT(c == infoCount);
    }

    void Add(neStackInfo *add) {
        if (!head) {
            head = tail = add;

            ASSERT(((neStackInfoItem *) add)->next == NULL);
        } else {
            ASSERT(add != tail);

            ((neStackInfoItem *) tail)->Append((neStackInfoItem *) add);

            tail = add;
        }
        infoCount++;

        add->stackHeader = this;
    }

    void Remove(neStackInfo *add, s32 flag = 0) {
/*		if (infoCount == 1 && !isHeaderX && flag == 0)
			ASSERT(0);
*/
        neStackInfoItem *item = (neStackInfoItem *) add;

        if (head == add)
            head = (neStackInfo *) item->next;

        if (tail == add)
            tail = (neStackInfo *) item->prev;

        item->Remove();

        infoCount--;

        add->stackHeader = nullptr;
    }

    bool Check(neStackInfo *st) {
        s32 c = 0;

        neStackInfoItem *item = (neStackInfoItem *) head;

        while (item) {
            ASSERT(c < infoCount);

            c++;

            neStackInfo *sinfo = (neStackInfo *) item;

            ASSERT(sinfo->stackHeader == this);

            if (st == sinfo) {
                return true;
            }
            item = item->next;
        }
        return false;
    }

    bool CheckStackDisconnected();

    neRigidBody *GetBottomStackBody() {
        return nullptr;
/*		if (!head)
			return NULL;

		neStackInfoItem * item = (neStackInfoItem *) head;

		while (item)
		{
			neStackInfo * sinfo = (neStackInfo *) item;

			neRigidBody * body = sinfo->body;

			neStackInfo * nextSinfo = NULL;

			for (s32 i = 0; i < sinfo->restOnCount; i++)
			{
				ASSERT (sinfo->restOn[i].body);

				if (sinfo->restOn[i].body->stackInfo)
				{
					if (sinfo->restOn[i].body->stackHeader == NULL)
					{

					}
					else
					{
						nextSinfo = sinfo->restOn[i].body->stackInfo;
						break;
					}
				}
				else
				{
					//return sinfo->restOn[i].body;
				}
			}
			if (nextSinfo != NULL)
			{
				item = (neStackInfoItem *)nextSinfo;
			}
			else
			{
				return sinfo->body;
			}
		}
		ASSERT(0);
		return NULL;
*/    }

    void ChangeHeader(neStackHeader *newHeader);

    void AddToSolver(/*bool withConstraint*/);

    void AddToSolverNoConstraintHeader();

    void ResetRigidBodyFlag();
};

using neStackHeaderHeap = neDLinkList<neStackHeader>;

using neStackHeaderItem = neFreeListItem<neStackHeader>;

#endif //NE_STACK_H
