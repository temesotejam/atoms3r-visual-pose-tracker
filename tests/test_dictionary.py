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


if __name__ == "__main__":
    unittest.main()
