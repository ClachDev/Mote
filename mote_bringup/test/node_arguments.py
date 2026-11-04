"""Read a launch_ros Node's command-line arguments back as plain strings.

launch_ros keeps them in the private, name-mangled ``_Node__arguments`` as
normalized substitutions; this is the one place the tests depend on that.
"""


def node_arguments(action):
    return [
        arg[0].text if isinstance(arg, list) else str(arg)
        for arg in action.__dict__["_Node__arguments"]
    ]
