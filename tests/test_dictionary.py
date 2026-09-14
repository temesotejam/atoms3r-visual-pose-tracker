import unittest

ID0 = [0xB532, 0xEB48, 0x4CAD, 0x12D7]
ID1 = [0x0F9A, 0x6547, 0x59F0, 0xE2A6]


def to_matrix(value):
    return [
        [((value >> (15 - (r * 4 + c))) & 1) for c in range(4)]
        for r in range(4)
    ]


def pack(matrix):
    value = 0
    for row in matrix:
        for bit in row:
            value = (value << 1) | bit
    return value


def rotate_ccw(matrix):
    return [list(row) for row in zip(*matrix)][::-1]


def canonical_corner_indices_for_detected_rotation(rotation):
    # Geometric image corners are TL,TR,BR,BL. The dictionary rotation is the
    # CCW rotation of canonical bits observed in the image, so restoring the
    # physical marker's canonical corner order requires the opposite shift.
    shift = (4 - (rotation & 3)) & 3
    return [(i + shift) & 3 for i in range(4)]


class DictionaryRotationTest(unittest.TestCase):
    def check_rotations(self, words):
        m = to_matrix(words[0])
        for expected in words:
            self.assertEqual(pack(m), expected)
            m = rotate_ccw(m)

    def test_id0_open_cv_rotations(self):
        self.check_rotations(ID0)

    def test_id1_open_cv_rotations(self):
        self.check_rotations(ID1)

    def test_ids_are_separated(self):
        self.assertGreaterEqual((ID0[0] ^ ID1[0]).bit_count(), 4)

    def test_canonical_corner_reordering(self):
        # rotation=1 means the canonical marker appears 90 deg CCW in image:
        # canonical TL is therefore geometric BL, and so on.
        self.assertEqual(
            canonical_corner_indices_for_detected_rotation(0), [0, 1, 2, 3]
        )
        self.assertEqual(
            canonical_corner_indices_for_detected_rotation(1), [3, 0, 1, 2]
        )
        self.assertEqual(
            canonical_corner_indices_for_detected_rotation(2), [2, 3, 0, 1]
        )
        self.assertEqual(
            canonical_corner_indices_for_detected_rotation(3), [1, 2, 3, 0]
        )


if __name__ == "__main__":
    unittest.main()
