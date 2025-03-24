#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import re
import networkx as nx
import matplotlib.pyplot as plt
import heapq
from demo_nova_sanctum.msg import BuchiAutomaton as BuchiAutomatonMsg  

class BuchiAutomaton(Node):
    def __init__(self):
        super().__init__('buchi_automaton_node')

        # **Declare ROS Parameter for Never Claim Input**
        self.declare_parameter("never_claim", "")

        # **Publisher for the Büchi Automaton**
        self.publisher = self.create_publisher(BuchiAutomatonMsg, "/buchi_automaton", 10)

        # **Parse and Construct the Büchi Automaton**
        never_claim = self.get_parameter("never_claim").get_parameter_value().string_value
        self.states = set()
        self.initial_states = set()
        self.accepting_states = set()
        self.alphabet = set()
        self.transitions = {}

        if never_claim:
            self._parse_never_claim(never_claim)
            self.publish_buchi_automaton()

        self.timer= self.create_timer(4, self.publish_buchi_automaton)
        
    def _parse_never_claim(self, never_claim):
        """Parses the Never Claim format into a Büchi Automaton."""
        lines = never_claim.split("\n")
        current_state = None

        for line in lines:
            line = line.strip()

            # Detect state definitions
            state_match = re.match(r'([a-zA-Z0-9_]+):', line)
            if state_match:
                current_state = state_match.group(1)
                self.states.add(current_state)

                if "init" in current_state:
                    self.initial_states.add(current_state)

                if "accept" in current_state:
                    self.accepting_states.add(current_state)

                if current_state not in self.transitions:
                    self.transitions[current_state] = {}

            # Detect transitions
            transition_match = re.match(r'::\s*\((.*?)\)\s*->\s*goto\s*([a-zA-Z0-9_]+)', line)
            if transition_match:
                condition, target_state = transition_match.groups()
                condition = condition.strip() if condition else "true"

                self.alphabet.update(re.findall(r'[a-zA-Z0-9_!]+', condition))

                if condition not in self.transitions[current_state]:
                    self.transitions[current_state][condition] = set()
                self.transitions[current_state][condition].add(target_state)

    def publish_buchi_automaton(self):
        """Publishes the constructed Büchi Automaton along with the accepting run (prefix & suffix)."""
        prefix_path, suffix_path = self.find_accepting_run()

        msg = BuchiAutomatonMsg()
        msg.states = list(self.states)
        msg.initial_states = list(self.initial_states)
        msg.accepting_states = list(self.accepting_states)
        msg.alphabet = list(self.alphabet)
        msg.transitions = [
            f"{state},{condition},{','.join(targets)}"
            for state, trans in self.transitions.items()
            for condition, targets in trans.items()
        ]
        msg.prefix_path = prefix_path
        msg.suffix_path = suffix_path

        self.publisher.publish(msg)
        # self.get_logger().info("Büchi Automaton published successfully.")
        # self.get_logger().info(f"Prefix Path: {prefix_path}")
        # self.get_logger().info(f"Suffix Path: {suffix_path}")

    def find_accepting_run(self):
        """Finds an accepting run satisfying the Büchi acceptance condition."""
        pq = []
        predecessors = {}
        min_distance = {state: float('inf') for state in self.states}

        for state in self.initial_states:
            heapq.heappush(pq, (0, state))
            predecessors[state] = None
            min_distance[state] = 0

        accepting_state = None

        while pq:
            curr_cost, state = heapq.heappop(pq)

            if state in self.accepting_states:
                accepting_state = state
                break

            for _, successors in self.transitions.get(state, {}).items():
                for successor in successors:
                    new_cost = curr_cost + 1
                    if new_cost < min_distance[successor]:
                        min_distance[successor] = new_cost
                        heapq.heappush(pq, (new_cost, successor))
                        predecessors[successor] = state

        if not accepting_state:
            print("\n No accepting path found.")
            return [], []

        # **Reconstruct Prefix Path**
        prefix_path = []
        state = accepting_state
        while state is not None:
            prefix_path.append(state)
            state = predecessors[state]
        prefix_path.reverse()

        # **Find Accepting Cycle**
        suffix_path = self._find_accepting_cycle(accepting_state)

        if not suffix_path:
            print("\n No valid Büchi accepting cycle found.")
            return [], []

        return prefix_path, suffix_path
    
    
    

    def _find_accepting_cycle(self, accepting_state):
        """Finds a cycle from an accepting state that satisfies Büchi conditions."""
        stack = []
        visited = {}
        accepting_cycle = []

        def dfs(current, path):
            nonlocal accepting_cycle
            if current in visited:
                if current in path:
                    cycle_start = path.index(current)
                    cycle = path[cycle_start:]
                    if any(state in self.accepting_states for state in cycle):
                        accepting_cycle.extend(cycle)
                return

            visited[current] = len(path)
            path.append(current)

            for _, successors in self.transitions.get(current, {}).items():
                for successor in successors:
                    dfs(successor, path[:])

        dfs(accepting_state, [])

        return accepting_cycle if accepting_cycle else []

def main(args=None):
    rclpy.init(args=args)
    node = BuchiAutomaton()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
