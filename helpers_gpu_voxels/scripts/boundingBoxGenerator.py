#!/usr/bin/env python

import numpy as np
from stl import mesh
import sys, os

class BoundingBoxGenerator():
    # constructor
    def __init__(self, extendBBcm = 0.0):
        self.extendBBcm = extendBBcm
        self.extendBB = extendBBcm*10#/100.0

    def __getMinMax(self):
        # min-max points
        self.minX = self.meshIn.x.min()
        self.maxX = self.meshIn.x.max()
        self.minY = self.meshIn.y.min()
        self.maxY = self.meshIn.y.max()
        self.minZ = self.meshIn.z.min()
        self.maxZ = self.meshIn.z.max()

        # BB length
        self.lenX = self.maxX - self.minX
        self.lenY = self.maxY - self.minY
        self.lenZ = self.maxZ - self.minZ

    def loadSTL(self, pathSTL):
        # load mesh
        self.meshIn = mesh.Mesh.from_file(pathSTL)
        print "Loaded STL: " + pathSTL

        # get file name (without extension) and folder path
        folder, stlFile = os.path.split(pathSTL)
        self.folder = folder
        self.filename = os.path.splitext(stlFile)[0]

    def getMeshBoundingBoxInfo(self, stl):
        print "x = [" + str(stl.x.min()) + ", " + str(stl.x.max()) + "], length: " + str(stl.x.max() - stl.x.min())
        print "y = [" + str(stl.y.min()) + ", " + str(stl.y.max()) + "], length: " + str(stl.y.max() - stl.y.min())
        print "z = [" + str(stl.z.min()) + ", " + str(stl.z.max()) + "], length: " + str(stl.z.max() - stl.z.min())

    def createBoundingBoxMesh(self):
        # info
        print "Bounding box of input file: " + self.meshIn.name
        self.getMeshBoundingBoxInfo(self.meshIn)
        self.__getMinMax()

        # create BB mesh - vertices of BB
        vertices = np.array([
            [self.minX - self.extendBB, self.minY - self.extendBB, self.minZ - self.extendBB],
            [self.minX - self.extendBB, self.maxY + self.extendBB, self.minZ - self.extendBB],
            [self.maxX + self.extendBB, self.minY - self.extendBB, self.minZ - self.extendBB],
            [self.maxX + self.extendBB, self.maxY + self.extendBB, self.minZ - self.extendBB], 
            [self.minX - self.extendBB, self.minY - self.extendBB, self.maxZ + self.extendBB],
            [self.minX - self.extendBB, self.maxY + self.extendBB, self.maxZ + self.extendBB],
            [self.maxX + self.extendBB, self.minY - self.extendBB, self.maxZ + self.extendBB],
            [self.maxX + self.extendBB, self.maxY + self.extendBB, self.maxZ + self.extendBB]
        ])
        
        # triangles of BB - 12 triangles (2 triangles per face -> 6x2 = 12) 
        # number in array "triangles" represents vertex from array "vertices" (0 - firts vertex, 1 - second vertex ...)
        triangles = np.array([
            [0, 1, 2],
            [2, 1, 3],
            [0, 4, 5],
            [0, 5, 1],
            [4, 6, 7],
            [4, 7, 5],
            [6, 2, 3],
            [6, 3, 7],
            [3, 1, 7],
            [1, 5, 7],
            [0, 2, 6],
            [0, 6, 4]
        ])

        # create mesh from extended BB
        self.meshOut = mesh.Mesh(np.zeros(triangles.shape[0], dtype = mesh.Mesh.dtype))

        # fill mesh with triangles (substitute triangle vertices with real values from array "vertices")
        for row, triangle in enumerate(triangles):
            for column in range(3):
                self.meshOut.vectors[row][column] = vertices[triangle[column]]
        
        # print info for extended mesh
        print "Mesh of extended bounding box: "
        self.getMeshBoundingBoxInfo(self.meshOut)

    def saveOutputMesh(self, pathToFolder = "default"):
        # get save name
        if pathToFolder == "default":
            saveName = self.folder + "/" + self.filename + "BB" + str(self.extendBBcm) + ".stl"
        else:
            saveName = pathToFolder + "/" + self.filename + "BB" + str(self.extendBBcm) + ".stl"
        
        # save mesh file
        self.meshOut.save(saveName)
        print "Saved STL: " + saveName

if __name__ == "__main__":
    bb = BoundingBoxGenerator(3)

    """imeStl = ["/home/larics/Documents/binvox_test/linkovi_stl/base_link.stl", 
              "/home/larics/Documents/binvox_test/linkovi_stl/link_1.stl",
              "/home/larics/Documents/binvox_test/linkovi_stl/link_2.stl",
              "/home/larics/Documents/binvox_test/linkovi_stl/link_3.stl",
              "/home/larics/Documents/binvox_test/linkovi_stl/link_4.stl",
              "/home/larics/Documents/binvox_test/linkovi_stl/link_5.stl",
              "/home/larics/Documents/binvox_test/linkovi_stl/link_6.stl"] """
    
    imeStl = ["/home/larics/Documents/binvox_test/linkovi_stl/ENDORSE_tool_v2.stl"]

    for ime in imeStl:
        bb.loadSTL(ime)
        bb.createBoundingBoxMesh()
        bb.saveOutputMesh()
        print ""
    