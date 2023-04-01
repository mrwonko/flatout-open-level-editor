# FlatOut Open Level Editor

Here's a cool idea: what if we could build our own levels for FlatOut 2?

In recent years, [Blender](https://blender.org) has evolved into a powerful customizable platform.
I think it would make a suitable base for a level editor.

As I explore that possibility, I will use this repository to collect my research, experiments and results.

Please don't expect too much. I should have the necessary programming skills, but sticking to large hobby projects has never been a strength of mine, especially not since I started working. I will try to document my work well enough for somebody else to take over some day.

## track_cdb2.gen import

A new import operator for the collision mesh now imports all relevant data into Blender.

I don't yet fully understand the per-triangle data. Maybe what I currently treat as a set of 6+2 bit flags is actually another 6-bit index and just 2 bit flags? Once I have the exporter, I can start experimenting to find out.

There's an optional Debug Visualisation of the collision tree in the first third of the file, which is very helpful for understanding the file format, but may freeze Blender temporarily while it creates a very large amount of objects. 
