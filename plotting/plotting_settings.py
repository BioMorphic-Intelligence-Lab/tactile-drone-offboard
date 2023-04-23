import matplotlib

matplotlib.rcParams["text.usetex"] = True
matplotlib.rcParams["font.family"] = "serif"

# Define plot resolution and size
dpi = 250
size = (20, 10)
text_size = 24
lw = 3


# Define the colors being used in the plots
colors = {"ee": "#01665e",
          "base": "#5ab587",
          "x": "#8c510a", 
          "y": "#d8b365",
          "z": "#ffcc33",
          "yaw": "#b23106",
          "wall_yaw": "#f1a340",
          "force" :"#e62e00"}