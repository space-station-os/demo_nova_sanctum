# List of states in the Büchi Automaton
string[] states

# Initial states
string[] initial_states

# Accepting states
string[] accepting_states

# Atomic propositions (input alphabet)
string[] alphabet

# Transitions (formatted as "state,condition,target_states")
string[] transitions

# Accepting run: Prefix path (initial state → accepting state)
string[] prefix_path

# Accepting run: Suffix path (loop in accepting state)
string[] suffix_path
