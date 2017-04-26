from svgpathtools import svg2paths, wsvg
paths, attributes = svg2paths("brush_stroke_calibration_curves.svg")
# Let's print out the first path object and the color it was in the SVG
# We'll see it is composed of two CubicBezier objects and, in the SVG file it
# came from, it was red
redpath = paths[0]

for path in paths:
	brushstroke = "1; "
	last = len(path) - 1
	for i, segment in enumerate(path):
		for control_point in segment:
			brushstroke += "{x}, {y}, ".format(x=control_point.real, y=control_point.imag)
	print brushstroke[:-2]