import rospy
import time
import sys
from rqt_graph import dotcode
from qt_dotgraph.pydotfactory import PydotFactory
import rosgraph.impl.graph


def main():
    rospy.init_node('graph_generator', sys.argv)

    graph = rosgraph.impl.graph.Graph()
    graph.set_master_stale(0.7)
    graph.set_node_stale(0.7)

    time.sleep(0.5)
    graph.update()

    graph_generator = dotcode.RosGraphDotcodeGenerator()
    code = graph_generator.generate_dotcode(
        rosgraphinst=graph,
        ns_filter="",
        topic_filter="",
        graph_mode='node_topic_all',  #node_node, node_topic, or node_topic_all
        hide_single_connection_topics=False,
        hide_dead_end_topics=False,
        cluster_namespaces_level=1,
        accumulate_actions=True,
        dotcode_factory=PydotFactory(),
        orientation='LR',
        unreachable=True,
        quiet=True)

    print(code)


if __name__ == '__main__':
    main()
