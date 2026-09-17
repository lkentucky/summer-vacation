"""Lossless triangle-mesh conversion to a millimeter-unit 3MF Core package."""
import collections
import json
import math
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET
import zipfile

source, target = map(pathlib.Path, sys.argv[1:3])
assert not target.exists(), f"Refusing to overwrite {target}"
data = source.read_bytes()
count = struct.unpack_from("<I", data, 80)[0]
assert len(data) == 84 + count * 50, "Expected binary STL"
vertices, indices, faces = [], {}, []
for i in range(count):
    face = []
    for j in range(3):
        point = struct.unpack_from("<3f", data, 84 + i * 50 + 12 + j * 12)
        assert all(map(math.isfinite, point))
        if point not in indices:
            indices[point] = len(vertices)
            vertices.append(point)
        face.append(indices[point])
    assert len(set(face)) == 3, "Degenerate triangle"
    faces.append(tuple(face))

edges = collections.Counter()
directions = collections.Counter()
for face in faces:
    for a, b in zip(face, face[1:] + face[:1]):
        key = tuple(sorted((a, b)))
        edges[key] += 1
        directions[key] += 1 if a < b else -1
assert all(n == 2 for n in edges.values()), "Mesh is not watertight"
assert all(n == 0 for n in directions.values()), "Inconsistent triangle winding"

ns = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
ET.register_namespace("", ns)
def tag(name):
    return f"{{{ns}}}{name}"
model = ET.Element(tag("model"), {"unit": "millimeter", "{http://www.w3.org/XML/1998/namespace}lang": "en-US"})
resources = ET.SubElement(model, tag("resources"))
obj = ET.SubElement(resources, tag("object"), {"id": "1", "type": "model", "name": source.stem})
mesh = ET.SubElement(obj, tag("mesh"))
vs = ET.SubElement(mesh, tag("vertices"))
for vertex in vertices:
    ET.SubElement(vs, tag("vertex"), dict(zip(("x", "y", "z"), map(repr, vertex))))
ts = ET.SubElement(mesh, tag("triangles"))
for face in faces:
    ET.SubElement(ts, tag("triangle"), dict(zip(("v1", "v2", "v3"), map(str, face))))
build = ET.SubElement(model, tag("build"))
ET.SubElement(build, tag("item"), {"objectid": "1"})

content_types = b'''<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>'''
relationships = b'''<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rel0" Target="/3D/3dmodel.model" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>'''
with zipfile.ZipFile(target, "x", compression=zipfile.ZIP_DEFLATED) as package:
    package.writestr("[Content_Types].xml", content_types)
    package.writestr("_rels/.rels", relationships)
    package.writestr("3D/3dmodel.model", ET.tostring(model, encoding="utf-8", xml_declaration=True))
with zipfile.ZipFile(target) as package:
    assert package.testzip() is None
    restored = ET.fromstring(package.read("3D/3dmodel.model"))
    assert restored.attrib["unit"] == "millimeter"
    restored_vertices = [tuple(float(v.attrib[c]) for c in ("x", "y", "z")) for v in restored.iter(tag("vertex"))]
    restored_faces = [tuple(int(f.attrib[c]) for c in ("v1", "v2", "v3")) for f in restored.iter(tag("triangle"))]
    assert restored_vertices == vertices and restored_faces == faces, "Geometry changed during conversion"
bounds = [max(v[k] for v in vertices) - min(v[k] for v in vertices) for k in range(3)]
print(json.dumps({"file": str(target), "vertices": len(vertices), "triangles": count, "unit": "millimeter", "dimensions_mm": bounds, "watertight": True, "geometry_preserved": True}, ensure_ascii=False))
