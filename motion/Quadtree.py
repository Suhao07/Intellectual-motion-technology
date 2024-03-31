class QuadtreeNode:
    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.children = [None] * 4  # NW, NE, SW, SE
        self.objects = []

class Quadtree:
    def __init__(self, x_min, x_max, y_min, y_max, max_objects=10, max_levels=8):
        self.root = QuadtreeNode(x_min, x_max, y_min, y_max)
        self.max_objects = max_objects
        self.max_levels = max_levels

    def clear(self):
        self.root.objects = []
        for i in range(4):
            if self.root.children[i]:
                self._clear_node(self.root.children[i])

    def _clear_node(self, node):
        node.objects = []
        for i in range(4):
            if node.children[i]:
                self._clear_node(node.children[i])

    def insert(self, obj, node=None, level=0):
        if node is None:
            node = self.root

        if len(node.objects) < self.max_objects and level < self.max_levels:
            node.objects.append(obj)
        else:
            if node.children[0] is None:
                self._split_node(node)

            quadrant = self._get_quadrant(obj, node)
            self.insert(obj, node.children[quadrant], level + 1)

    def _split_node(self, node):
        x_mid = (node.x_min + node.x_max) / 2
        y_mid = (node.y_min + node.y_max) / 2

        node.children[0] = QuadtreeNode(node.x_min, x_mid, y_mid, node.y_max)  # NW
        node.children[1] = QuadtreeNode(x_mid, node.x_max, y_mid, node.y_max)  # NE
        node.children[2] = QuadtreeNode(node.x_min, x_mid, node.y_min, y_mid)  # SW
        node.children[3] = QuadtreeNode(x_mid, node.x_max, node.y_min, y_mid)  # SE

        for obj in node.objects:
            quadrant = self._get_quadrant(obj, node)
            node.children[quadrant].objects.append(obj)
        node.objects = []

    def _get_quadrant(self, obj, node):
        x_mid = (node.x_min + node.x_max) / 2
        y_mid = (node.y_min + node.y_max) / 2
        if obj.x <= x_mid:
            if obj.y <= y_mid:
                return 0  # NW
            else:
                return 2  # SW
        else:
            if obj.y <= y_mid:
                return 1  # NE
            else:
                return 3  # SE

    def query_range(self, range_x_min, range_x_max, range_y_min, range_y_max):
        return self._query_range(self.root, range_x_min, range_x_max, range_y_min, range_y_max)

    def _query_range(self, node, range_x_min, range_x_max, range_y_min, range_y_max):
        if node is None:
            return []

        result = []
        if range_x_max >= node.x_min and range_x_min <= node.x_max and range_y_max >= node.y_min and range_y_min <= node.y_max:
            for obj in node.objects:
                if range_x_min <= obj.x <= range_x_max and range_y_min <= obj.y <= range_y_max:
                    result.append(obj)

            for child_node in node.children:
                result.extend(self._query_range(child_node, range_x_min, range_x_max, range_y_min, range_y_max))

        return result
