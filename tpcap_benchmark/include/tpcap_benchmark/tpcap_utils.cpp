#include "tpcap_utils.h"

namespace TPCAP
{
    void fillPolygon(const std::vector<Point> &vertices, double resolution, std::vector<intPoint> &polygArea)
    {
        if (vertices.size() < 3)
            return;
        std::vector<intPoint> intVertices;
        for (int i = 0; i < vertices.size(); i++)
        {
            intPoint intVertex = {(int)(vertices[i].x / resolution), (int)(vertices[i].y / resolution)};
            intVertices.push_back(intVertex);
        }
        std::vector<int> tempY;
        int minY, maxY, scanLineNums;
        for (int i = 0; i < intVertices.size(); i++)
        {
            tempY.push_back(intVertices[i].y);
        }
        minY = *std::min_element(tempY.begin(), tempY.end());
        maxY = *std::max_element(tempY.begin(), tempY.end());
        scanLineNums = maxY - minY + 1;
        /*建立边表*/
        EdgeTable ET = new Edge *[scanLineNums];
        for (int y = 0; y < scanLineNums; y++)
        {
            ET[y] = new Edge;
            ET[y]->next = NULL;
        }
        intPoint predP, currP1, currP2, nextP;
        /*初始化边表*/
        for (int i = 0; i < intVertices.size(); i++)
        {
            currP1.y = intVertices[i].y;
            currP2.y = intVertices[(i + 1) % intVertices.size()].y;
            if (currP1.y == currP2.y) //舍弃平行X轴的边
                continue;
            currP1.x = intVertices[i].x;
            currP2.x = intVertices[(i + 1) % intVertices.size()].x;
            predP.x = intVertices[(i - 1 + intVertices.size()) % intVertices.size()].x;
            predP.y = intVertices[(i - 1 + intVertices.size()) % intVertices.size()].y;
            nextP.x = intVertices[(i + 2) % intVertices.size()].x;
            nextP.y = intVertices[(i + 2) % intVertices.size()].y;
            int ymin = std::min(currP1.y, currP2.y);
            int ymax = std::max(currP1.y, currP2.y);
            double x = currP1.y > currP2.y ? currP2.x : currP1.x;
            double dx = (double)(currP1.x - currP2.x) / (double)(currP1.y - currP2.y);
            if (((currP2.y >= currP1.y) && (currP1.y >= predP.y)) || ((currP1.y >= currP2.y) && (currP2.y >= nextP.y)))
            {
                ymin++;
                x += dx;
            }
            Edge *tempE = new Edge;
            tempE->ymax = ymax;
            tempE->x = x;
            tempE->dx = dx;
            tempE->next = ET[ymin - minY]->next;
            ET[ymin - minY]->next = tempE;
        }
        /*建立活动边表*/
        ActiveEdgeTable AET = new Edge;
        AET->next = NULL;
        /*扫描线扫描*/
        for (int y = minY; y < maxY + 1; y++)
        {
            /*取出ET中当前扫描行的所有边并按x的递增顺序（若x相等则按dx的递增顺序）插入AET*/
            while (ET[y - minY]->next)
            {
                Edge *tempE = ET[y - minY]->next;
                Edge *tempAET = AET;
                while (tempAET->next)
                {
                    if ((tempE->x > tempAET->next->x) || ((tempE->x == tempAET->next->x) && (tempE->dx > tempAET->next->dx)))
                    {
                        tempAET = tempAET->next;
                        continue;
                    }
                    break;
                }
                ET[y - minY]->next = tempE->next;
                tempE->next = tempAET->next;
                tempAET->next = tempE;
            }
            /*将AET中的边两两配对并将中间点添加到polygArea中*/
            Edge *tempAET = AET;
            while (tempAET->next && tempAET->next->next)
            {
                for (int x = tempAET->next->x; x < tempAET->next->next->x; x++)
                {
                    intPoint occ = {x, y};
                    polygArea.push_back(occ);
                }
                tempAET = tempAET->next->next;
            }
            /*删除AET中满足y=ymax的边*/
            tempAET = AET;
            while (tempAET->next)
            {
                if (tempAET->next->ymax == y)
                {
                    Edge *tempE = tempAET->next;
                    tempAET->next = tempE->next;
                    tempE->next = NULL;
                    delete tempE;
                }
                else
                {
                    tempAET = tempAET->next;
                }
            }
            /*更新AET中边的x值，进入下一循环*/
            tempAET = AET;
            while (tempAET->next)
            {
                tempAET->next->x += tempAET->next->dx;
                tempAET = tempAET->next;
            }
        }
        /*释放边表内存*/
        for (int y = 0; y < scanLineNums; y++)
        {
            while (ET[y]->next)
            {
                Edge *next = ET[y]->next->next;
                delete ET[y]->next;
                ET[y]->next = next;
            }
            delete ET[y];
        }
        delete[] ET;
        /*释放活动边表内存*/
        while (AET)
        {
            Edge *next = AET->next;
            delete AET;
            AET = next;
        }
    }
}