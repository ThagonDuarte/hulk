import unittest

from ultralytics_dfine.schemas import (
    DHRP_FLIP_IDX,
    FIELD_FEATURE_SCHEMA,
    PERSON_POSE_SCHEMA,
    ROBOT_POSE_SCHEMA,
    SCHEMA_REGISTRY,
    HeadId,
)


class SchemaRegistryTests(unittest.TestCase):
    def test_registry_uses_unique_head_ids(self) -> None:
        self.assertEqual(
            set(SCHEMA_REGISTRY),
            {
                HeadId.PERSON_POSE,
                HeadId.ROBOT_POSE,
                HeadId.FIELD_FEATURES,
            },
        )

    def test_pose_schemas_have_expected_shapes(self) -> None:
        self.assertEqual(PERSON_POSE_SCHEMA.keypoint_count, 17)
        self.assertEqual(ROBOT_POSE_SCHEMA.keypoint_count, 14)
        self.assertTrue(ROBOT_POSE_SCHEMA.oks_uses_direct_k)

    def test_dhrp_flip_is_an_involution(self) -> None:
        flipped_twice = tuple(DHRP_FLIP_IDX[index] for index in DHRP_FLIP_IDX)
        self.assertEqual(flipped_twice, tuple(range(14)))

    def test_field_schema_has_fixed_queries(self) -> None:
        self.assertEqual(FIELD_FEATURE_SCHEMA.num_queries, 300)
        self.assertEqual(len(FIELD_FEATURE_SCHEMA.class_names), 5)
        self.assertTrue(FIELD_FEATURE_SCHEMA.implicit_background)


if __name__ == "__main__":
    unittest.main()
