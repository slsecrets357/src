#!/usr/bin/env python3

import networkx as nx
import numpy as np
import cv2
import os

class track_map():
    def __init__(self,posX,posY,rot,path):
        self.map = cv2.imread(os.path.dirname(os.path.realpath(__file__))+'/map.png')
        self.map_graph = nx.DiGraph()
        self.locations = ['start','int1N','int1S','int1W','int1E',
        'int2N','int2S','int2W','int2E','int3N','int3S','int3W','int3E',
        'int4N','int4S','int4W','int4E','int5N','int5W','int5E',
        'int6N','int6S','int6W','int6E','track1N','track1S','track2N',
        'track2S','track3N','track3S','parkingN',
        'parkingS','roundabout','highwayN','highwayS','curvedpath']
        self.locations_coord = [[0.75,14.25],[0.75,12.25],[0.75,12.25],
        [1.5,13.75],[1.5,13.75],[3,12.25],[3,12.25],[4,13.75],[4,13.75],
        [0.75,9],[0.75,9],[1.5,10.75],[1.5,10.75],[5,12.25],[5,12.25],
        [4,10.75],[4,10.75],[3,9],[1.5,7.25],[1.5,7.25],[5,9],[5,9],
        [4,7.25],[4,7.25],[1.5,4],[1.5,4],[9.5,2.25],[9.5,2.25],[6.5,5.5],
        [6.5,5.5],[4,2.25],[4,2.25],[9.5,4],[10,10],[10,10],[12,12]]
        self.location_decision_points = [[0.83,14.53],[0.83,11.63],
        [0.45,12.56],[1.63,13.32],[1.93,13.71],
        [3.05,11.63],[2.67,12.56],[3.83,13.32],[4.13,13.71],
        [0.83,7.83],[0.45,9.69],[1.63,10.44],[1.93,10.83],
        [5.28,11.63],[4.9,12.56],[3.83,10.44],[4.13,10.83],
        [3.05,7.83],[1.63,6.61],[1.93,7],
        [5.28,7.83],[4.9,9.69],[3.83,6.61],[4.13,7],
        [2.7,2],[0.45,5.83],[11,3.5],[4.6,1.7],[4.9,5.87],[8.56,3.92],
        [4.3,2.1],[3,1.7],[0,0],[10,5],[6,13],[11.9,4.7]
        ]
        self.planned_path = path
        
        # graph map creation (run to get edgelist)
        # self.map_graph.add_nodes_from(self.locations)
        # for i in range(len(self.locations_coord)):
        #     self.map_graph.nodes[self.locations[i]]['coord']=self.locations_coord[i]
        # self.add_all_edges()
        # nx.write_edgelist(self.map_graph,os.path.dirname(os.path.realpath(__file__))+'/map.edgelist')

        # loading the graph map
        self.map_graph=nx.read_edgelist(os.path.dirname(os.path.realpath(__file__))+'/map.edgelist',create_using=nx.DiGraph())
        for i in range(len(self.map_graph.nodes)):
            self.map_graph.nodes[self.locations[i]]['coord']=self.locations_coord[i]

        # get the current location
        self.location = ''
        self.locate(posX,posY,rot)

        # calculate the shortest path
        self.path = []
        self.directions = []
        self.plan_path()

    def plan_path(self):
        # get the path and directions
        self.path = [self.location]
        loc = self.location
        for i in range(len(self.planned_path)):
            p = nx.shortest_path(self.map_graph, source=loc, target=self.planned_path[i], weight='weight')
            loc = self.planned_path[i]
            self.path += p[1:]
        self.directions = []
        for i in range(len(self.path)-1):
            d=self.map_graph.get_edge_data(self.path[i],self.path[i+1]).get('dir')
            self.directions.append(d)
        # print(self.path)
        # print(self.directions)

    def draw_map(self):
        # draw the path
        img_map=self.map
        cv2.circle(img_map, (int(self.map_graph.nodes[self.location]['coord'][0]/15*self.map.shape[0]),int(self.map_graph.nodes[self.location]['coord'][1]/15*self.map.shape[1])), radius=20, color=(0,255,0), thickness=-1)
        for i in range(len(self.map_graph.nodes)):
            cv2.putText(img_map, self.locations[i], (int(self.map_graph.nodes[self.locations[i]]['coord'][0]/15*self.map.shape[0]),int(self.map_graph.nodes[self.locations[i]]['coord'][1]/15*self.map.shape[1])), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0,0,255), 3, cv2.LINE_AA)
        for i in range(len(self.path)-1):
            img_map = cv2.arrowedLine(img_map, (int(self.map_graph.nodes[self.path[i]]['coord'][0]/15*self.map.shape[0]),int(self.map_graph.nodes[self.path[i]]['coord'][1]/15*self.map.shape[1])),
            (int(self.map_graph.nodes[self.path[i+1]]['coord'][0]/15*self.map.shape[0]),int(self.map_graph.nodes[self.path[i+1]]['coord'][1]/15*self.map.shape[1])), color=(255,0,255), thickness=10)
        windowName = 'track'
        cv2.namedWindow(windowName,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName,700,700)
        cv2.imshow(windowName, img_map)
        key = cv2.waitKey(0)

    def decision_point(self,x,y):
        loc = self.location_decision_points[self.locations.index(self.location)]
        d = (loc[0]-x)**2+(loc[1]-y)**2
        if d<0.25:
            return True
        else:
            return False

    def locate(self,x,y,rot):
        # get current location based on position and orientation
        if y>6.2:
            if x<1.3:
                if y<10.75:
                    if rot<np.pi and rot>0:
                        self.location = 'int3N'
                    else:
                        self.location = 'int3S'
                elif y<13.7:
                    if rot<np.pi and rot>0:
                        self.location = 'int1N'
                    else:
                        self.location = 'int1S'
                else:
                    self.location = 'start'
            elif x<2.55:
                if y<8.75:
                    if rot<np.pi/2 and rot>-np.pi/2:
                        self.location = 'int5E'
                    else:
                        self.location = 'int5W'
                elif y<12.25:
                    if rot<np.pi/2 and rot>-np.pi/2:
                        self.location = 'int3E'
                    else:
                        self.location = 'int3W'
                else:
                    if rot<np.pi/2 and rot>-np.pi/2:
                        self.location = 'int1E'
                    else:
                        self.location = 'int1W'
            elif x<3.25:
                if y<10.75:
                    # no int5S
                    self.location = 'int5N'
                else:
                    if rot<np.pi and rot>0:
                        self.location = 'int2N'
                    else:
                        self.location = 'int2S'
            elif x<4.65:
                if y<8.75:
                    if rot<np.pi/2 and rot>-np.pi/2:
                        self.location = 'int6E'
                    else:
                        self.location = 'int6W'
                elif y<12.25:
                    if rot<np.pi/2 and rot>-np.pi/2:
                        self.location = 'int4E'
                    else:
                        self.location = 'int4W'
                else:
                    if rot<np.pi/2 and rot>-np.pi/2:
                        self.location = 'int2E'
                    else:
                        self.location = 'int2W'
            elif x<5.5:
                if y<10.75:
                    if rot<np.pi and rot>0:
                        self.location = 'int6N'
                    else:
                        self.location = 'int6S'
                else:
                    if rot<np.pi and rot>0:
                        self.location = 'int4N'
                    else:
                        self.location = 'int4S'
            elif x>11:
                self.location = 'curvedpath'
            else:
                if rot<3/4*np.pi and rot>-1/4*np.pi:
                    self.location = 'highwayN'
                else:
                    self.location = 'highwayS'
        else:
            if x<2.75:
                if rot<3/4*np.pi and rot>-1/4*np.pi:
                    self.location = 'track1N'
                else:
                    self.location = 'track1S'
            elif x<4.75:
                if rot<3/4*np.pi and rot>-1/4*np.pi:
                    self.location = 'parkingN'
                else:
                    self.location = 'parkingS'
            elif y<2.9:
                if x<13:
                    if rot<3/4*np.pi and rot>-1/4*np.pi:
                        self.location = 'track2N'
                    else:
                        self.location = 'track2S'
                else:
                    if rot<1/4*np.pi and rot>-3/4*np.pi:
                        self.location = 'track2N'
                    else:
                        self.location = 'track2S'
            elif x<8.6:
                if rot<3/4*np.pi and rot>-1/4*np.pi:
                    self.location = 'track3N'
                else:
                    self.location = 'track3S'
            elif y<4.6:
                if x<10.75:
                    self.location = 'roundabout'
                else:
                    if rot<1/2*np.pi and rot>-1/2*np.pi:
                        self.location = 'track2S'
                    else:
                        self.location = 'track2N'
            elif x>11:
                self.location = 'curvedpath'
            else:
                if rot<3/4*np.pi and rot>-1/4*np.pi:
                    self.location = 'highwayN'
                else:
                    self.location = 'highwayS'
        # print('location: '+self.location)
    
    # graph creation helpers
    
    def add_edge(self,x,y,d):
        w = (abs(self.map_graph.nodes[x]['coord'][0]-self.map_graph.nodes[y]['coord'][0])+
        abs(self.map_graph.nodes[x]['coord'][1]-self.map_graph.nodes[y]['coord'][1]))
        self.map_graph.add_edge(x,y,weight=w,dir=d)
    
    def add_all_edges(self):
        self.add_edge('start','int1N','straight')
        self.add_edge('start','int1E','right')
        self.add_edge('int1N','int3N','straight')
        self.add_edge('int1N','int3E','right')
        self.add_edge('int1S','start','straight')
        self.add_edge('int1S','int1E','left')
        self.add_edge('int1W','start','left')
        self.add_edge('int1W','int1N','right')
        self.add_edge('int1E','int2E','straight')
        self.add_edge('int1E','int2N','left')
        self.add_edge('int2N','int5N','straight')
        self.add_edge('int2N','int3W','left')
        self.add_edge('int2N','int4E','right')
        self.add_edge('int2S','int1W','left')
        self.add_edge('int2S','int2E','right')
        self.add_edge('int2W','int1W','straight')
        self.add_edge('int2W','int2N','right')
        self.add_edge('int2E','int4N','left')
        self.add_edge('int2E','highwayN','straight')
        self.add_edge('int3N','track1N','straight')
        self.add_edge('int3N','int5E','right')
        self.add_edge('int3S','int1S','straight')
        self.add_edge('int3S','int3E','left')
        self.add_edge('int3W','int1S','left')
        self.add_edge('int3W','int3N','right')
        self.add_edge('int3E','int5N','left')
        self.add_edge('int3E','int2S','right')
        self.add_edge('int3E','int4E','straight')
        self.add_edge('int4N','int6N','straight')
        self.add_edge('int4N','int4W','left')
        self.add_edge('int4S','int2W','right')
        self.add_edge('int4S','highwayN','left')
        self.add_edge('int4W','int2S','left')
        self.add_edge('int4W','int5N','right')
        self.add_edge('int4W','int3W','straight')
        self.add_edge('int4E','int4S','right')
        self.add_edge('int4E','int6N','left')
        self.add_edge('int5N','int6E','right')
        self.add_edge('int5N','int5W','left')
        self.add_edge('int5W','track1N','right')
        self.add_edge('int5W','int3S','left')
        self.add_edge('int5E','int6E','straight')
        self.add_edge('int6N','int6W','left')
        self.add_edge('int6N','track3N','straight')
        self.add_edge('int6S','int4W','right')
        self.add_edge('int6S','int4S','straight')
        self.add_edge('int6W','int5W','straight')
        self.add_edge('int6E','int6S','right')
        self.add_edge('int6E','track3N','left')
        self.add_edge('track1N','parkingN','straight')
        self.add_edge('track1S','int5E','left')
        self.add_edge('track1S','int3S','straight')
        self.add_edge('parkingN','track2N','straight')
        self.add_edge('parkingS','track1S','straight')
        self.add_edge('track2S','parkingS','straight')
        self.add_edge('track2N','roundabout','straight')
        self.add_edge('roundabout','track2S','-')
        self.add_edge('roundabout','highwayS','-')
        self.add_edge('roundabout','track3S','-')
        self.add_edge('track3N','roundabout','')
        self.add_edge('track3S','int6W','right')
        self.add_edge('track3S','int6S','straight')
        self.add_edge('highwayN','roundabout','straight')
        self.add_edge('highwayS','int4N','right')
        self.add_edge('highwayS','int2W','straight')
        self.add_edge('curvedpath','track2N','right')
        self.add_edge('curvedpath','roundabout','left')

if __name__ == '__main__':
    m = ['int1E','int2N','int5N','int6E','int6S','int4W','int3W','int1S','start']
    node = track_map(13.5,4.5,1.5,m)
    node.draw_map()