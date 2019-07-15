# !/usr/bin/env  python
# coding:utf-8

import numpy as np
from sklearn.cluster import KMeans


class Mesh:
    def __init__(self, world_state, objects):
        self.world_state = world_state
        self.robot_total = objects.robot_total
        self.enemy_total = objects.enemy_total
        self.robot = objects.robot
        self.enemy = objects.enemy

    """フィールドをメッシュ化する(クラスタリング用に書かれた)"""
    def create_mesh(self):
        self.mesh_size = 50
        self.mesh_x_num = int(self.world_state.field_x_size / (2*self.mesh_size)) + 1
        self.mesh_y_num = int(self.world_state.field_y_size / self.mesh_size) + 1
        self.mesh_x = [ i * self.mesh_size for i in range(self.mesh_x_num)]
        self.mesh_y = [- self.world_state.field_y_size / 2 + i * self.mesh_size for i in range(self.mesh_y_num)]
        #print(self.mesh_x)
        #print(self.mesh_y)
        self.mesh_xy = []
        for x in self.mesh_x:
            for y in self.mesh_y:
                self.mesh_xy.append([x, y])
        return self.mesh_xy

    """各メッシュの密度計算"""
    def enemy_density(self):
        mesh_xy = np.array(self.create_mesh())
        distance = np.zeros([self.mesh_x_num * self.mesh_y_num])
        enemy_position = []
        for i in range(self.robot_total-1):
            x, y, _ = self.enemy[i+1].get_current_position()
            #print(x,y)
            distance_ = (mesh_xy - np.array([x,y]))**2
            distance += np.sqrt(distance_[:,0] + distance_[:,1])
        mean_distance = distance / 7
        return mean_distance.reshape([self.mesh_x_num, self.mesh_y_num])

    """密度からk-meansでクラス境界を獲得"""
    def space_clustering(self):
        self.threshold = 3000
        self.n_cluster = 3
        mean_distance = self.enemy_density().reshape([self.mesh_x_num * self.mesh_y_num])
        mesh_xy = np.array(self.mesh_xy)
        plots = mesh_xy[mean_distance > self.threshold]
        print(plots.shape)
        cls = KMeans(self.n_cluster)
        pred = cls.fit_predict(plots)

        #"""
        for i in range(self.n_cluster):
            labels = plots[pred == i]
            plt.scatter(labels[:, 0], labels[:, 1])
            #"""
            centers = cls.cluster_centers_

        plt.scatter(centers[:, 0], centers[:, 1], s=100,facecolors='none', edgecolors='black')
        plt.xlim(-self.world_state.field_x_size/2, self.world_state.field_x_size/2)
        plt.ylim(-self.world_state.field_y_size/2, self.world_state.field_y_size/2)
        plt.show()
        #"""
