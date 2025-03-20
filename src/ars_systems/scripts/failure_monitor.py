#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import networkx as nx
import itertools
import heapq
from demo_nova_sanctum.msg import CdraStatus, BuchiAutomaton, FailureAlert

class ProductAutomaton(Node):
    def __init__(self):
        super().__init__('product_automaton_node')

        # **Initialize Automaton Components**
        self.ts_state = None
        self.buchi_automaton = None
        self.current_buchi_state = None
        self.product_states = set()
        self.initial_states = set()
        self.accepting_states = set()
        self.transitions = {}

        # **Subscribe to Transition System & Büchi Automaton**
        self.ts_subscriber = self.create_subscription(CdraStatus, "/cdra_status", self.ts_callback, 10)
        self.ba_subscriber = self.create_subscription(BuchiAutomaton, "/buchi_automaton", self.ba_callback, 10)

        # **Failure Alert Publisher**
        self.failure_publisher = self.create_publisher(FailureAlert, "/failure_alert", 10)

    def ts_callback(self, msg):
        """Handles incoming Transition System states from /cdra_status."""
        self.ts_state = str(msg.co2_processing_state)  # Convert to string for matching Büchi labels
        self.get_logger().info(f"[TS] Transition System Updated: {self.ts_state}")

        if self.buchi_automaton:
            self.update_product_automaton()
            self.check_for_failure()
            
    def get_buchi_successors(self, buchi_state, ts_state):
        """
        Returns successor states in the Büchi Automaton based on atomic propositions from the Transition System.
        """
        valid_states = set()

        # Extract atomic proposition labels from TS
        ts_labels = []
        if ts_state == "2":  # Threshold reached
            ts_labels.append("p3")
        elif ts_state == "3":  # Sending to desciccant
            ts_labels.append("p4")

        self.get_logger().info(f"[BA] Checking transitions for {buchi_state} with TS labels {ts_labels}")

        # Ensure the state exists in the Büchi Automaton dictionary
        if buchi_state not in self.buchi_automaton_transitions:
            self.get_logger().warning(f"[BA] State {buchi_state} not found in Büchi Automaton.")
            return valid_states

        # Check if any transition condition matches the TS labels
        for condition, targets in self.buchi_automaton_transitions[buchi_state].items():
            if any(label in condition for label in ts_labels):
                valid_states.update(targets)
                self.get_logger().info(f"[BA] Matched transition: {buchi_state} → {targets} via {condition}")

        if not valid_states:
            self.get_logger().warning(f"[BA] No matching transitions found for {buchi_state}!")

        return valid_states



    def get_ts_successors(self, ts_state):
        """Returns the successor states from the transition system based on the current TS state."""
        ts_successors = set()

        # Transition System state mappings (Ensure this matches your AirCollector states)
        transition_map = {
            "0": ["1"],  # IDLE → COLLECTING
            "1": ["2"],  # COLLECTING → THRESHOLD_REACHED
            "2": ["3"],  # THRESHOLD_REACHED → SENDING_TO_DESICCANT
            "3": ["0"]   # SENDING_TO_DESICCANT → IDLE
        }

        if ts_state in transition_map:
            ts_successors.update(transition_map[ts_state])

        return ts_successors

    def ba_callback(self, msg):
        """Handles incoming Büchi Automaton from /buchi_automaton."""
        self.buchi_automaton = msg

        # Convert transition list into a usable dictionary
        self.buchi_automaton_transitions = {}  # Use a new variable, not the msg object

        if not msg.transitions:
            self.get_logger().error("[BA] Received Büchi Automaton with NO transitions!")
            return
        if msg.initial_states:
            self.current_buchi_state = msg.initial_states[0]  # Pick first initial state
            self.get_logger().info(f"[BA] Initial Büchi state set to: {self.current_buchi_state}")
        else:
            self.get_logger().error("[BA] No initial state found in Büchi Automaton!")

        # Parse transitions into a dictionary
        for transition in msg.transitions:
            parts = transition.split(",")  # Format: "source,condition,target1,target2,..."

            if len(parts) < 3:
                self.get_logger().warning(f"[BA] Skipping malformed transition: {transition}")
                continue  # Skip incorrect entries

            source = parts[0].strip()
            condition = parts[1].strip()
            targets = {target.strip() for target in parts[2:]}  # Remaining parts are target states

            if source not in self.buchi_automaton_transitions:
                self.buchi_automaton_transitions[source] = {}

            if condition not in self.buchi_automaton_transitions[source]:
                self.buchi_automaton_transitions[source][condition] = set()

            self.buchi_automaton_transitions[source][condition].update(targets)

        self.get_logger().info(f"[BA] Parsed {len(self.buchi_automaton_transitions)} Büchi states.")

        if self.ts_state is not None:
            self.construct_product_automaton()
            self.find_accepting_run()
            self.update_product_automaton()
            self.check_for_failure()



    def construct_product_automaton(self):
        """Constructs the Product Automaton using the Transition System and Büchi Automaton."""
        self.product_states.clear()
        self.transitions.clear()
        self.initial_states.clear()
        self.accepting_states.clear()

        for ts_state, ba_state in itertools.product(self.ts_state, self.buchi_automaton.states):
            product_state = (ts_state, ba_state)
            self.product_states.add(product_state)

            # Identify initial states
            if ts_state == "0" and ba_state in self.buchi_automaton.initial_states:
                self.initial_states.add(product_state)

            # Identify accepting states
            if ba_state in self.buchi_automaton.accepting_states:
                self.accepting_states.add(product_state)

            # Compute valid transitions
            for ts_successor in self.get_ts_successors(ts_state):
                valid_ba_successors = self.get_buchi_successors(ba_state, ts_state)
                for ba_successor in valid_ba_successors:
                    new_product_state = (ts_successor, ba_successor)
                    if product_state not in self.transitions:
                        self.transitions[product_state] = set()
                    self.transitions[product_state].add(new_product_state)

    def find_accepting_run(self):
        """Finds an accepting run (prefix + cycle) in the Product Automaton."""
        pq = []
        predecessors = {}
        min_distance = {state: float('inf') for state in self.product_states}

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

            for successor in self.transitions.get(state, []):
                new_cost = curr_cost + 1
                if new_cost < min_distance[successor]:
                    min_distance[successor] = new_cost
                    heapq.heappush(pq, (new_cost, successor))
                    predecessors[successor] = state

        if not accepting_state:
            self.get_logger().warning("No accepting path found.")
            return [], []

        # **Reconstruct Prefix Path**
        prefix_path = []
        state = accepting_state
        while state is not None:
            prefix_path.append(state)
            state = predecessors[state]
        prefix_path.reverse()

        # **Find Accepting Cycle**
        suffix_path = self.find_accepting_cycle(accepting_state)

        if not suffix_path:
            self.get_logger().warning("No valid Büchi accepting cycle found.")
            return [], []

        return prefix_path, suffix_path

    def find_accepting_cycle(self, accepting_state):
        """Finds a cycle from an accepting state that revisits it infinitely often."""
        stack = []
        visited = set()
        cycle_path = []

        def dfs_cycle(state):
            if state in visited:
                if state in stack:
                    cycle_start = stack.index(state)
                    cycle_path.extend(stack[cycle_start:])
                return

            visited.add(state)
            stack.append(state)

            for successor in self.transitions.get(state, []):
                dfs_cycle(successor)

            stack.pop()

        dfs_cycle(accepting_state)
        return cycle_path if cycle_path else []

    def update_product_automaton(self):
        """Updates the Product Automaton dynamically as the transition system changes."""
        if not self.current_buchi_state:
            self.get_logger().warning("No initial Büchi state set.")
            return

        # Find valid Büchi transitions based on TS atomic proposition
        valid_ba_successors = self.get_buchi_successors(self.current_buchi_state, self.ts_state)

        if valid_ba_successors:
            self.current_buchi_state = list(valid_ba_successors)[0]
            self.get_logger().info(f"[PA] Transitioned to Product State: ({self.ts_state}, {self.current_buchi_state})")
        else:
            self.get_logger().warning(f"[ERROR] No valid Büchi transitions found for ({self.ts_state}, {self.current_buchi_state})")

    def check_for_failure(self):
        """Checks if the current product automaton state violates Büchi acceptance conditions."""
        product_state = (self.ts_state, self.current_buchi_state)

        if product_state in self.accepting_states:
            self.get_logger().info("[ACCEPT] System satisfies LTL specification.")
        else:
            self.publish_failure_alert()

    def publish_failure_alert(self):
        """Publishes an alert if the system violates the Büchi acceptance condition."""
        alert_msg = FailureAlert()
        alert_msg.alert_message = "ALERT: The system is NOT satisfying the safety conditions!"
        alert_msg.severity = 2  # Critical severity

        self.failure_publisher.publish(alert_msg)
        self.get_logger().error("[FAILURE] CRITICAL ALERT: System does not satisfy LTL constraints!")

def main(args=None):
    rclpy.init(args=args)
    node = ProductAutomaton()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
