// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut
// =============================================================================
//
// What a visual shape with no material of its own looks like.
//
// Chrono has one answer to that question, ChVisualMaterial::Default(), and every rendering
// backend is supposed to use it. The OptiX backend used to keep a second, hardcoded answer
// (Kd = 0.5 grey, roughness = 1), which drifted from the shared one when the shared one was
// corrected in 2022. The result was measurable: a material-less box rendered exactly 2x darker
// under OptiX than under the Vulkan RT backend, and, with no second backend involved at all,
// two standard Chrono box helpers rendered the same box differently, because
// chrono::utils::AddBoxGeometry attaches Default() while ChBodyEasyBox attaches nothing.
//
// These tests pin the shared answer. They are deliberately CPU-only and live outside the OptiX
// block in CMakeLists: they test the contract the backends must honour, not the GPU code that
// honours it, so they must also run in builds without OptiX. The OptiX side is covered by
// demo_SEN_vulkan_validation --probe-box-material, which needs a GPU.
//
// =============================================================================

#include <atomic>
#include <sstream>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/serialization/ChArchiveJSON.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// The default material's own values.
// -----------------------------------------------------------------------------

// The values a backend gets when it asks Chrono what "no material" looks like. Pinned as exact
// numbers rather than as "whatever the constructor says", because the whole defect was a second
// copy of these numbers drifting away from the first. A backend that hardcodes its own copy
// should have to change this test to stay green.
TEST(DefaultVisualMaterial, has_the_documented_appearance) {
    auto def = ChVisualMaterial::Default();
    ASSERT_NE(def, nullptr);

    EXPECT_FLOAT_EQ(def->GetDiffuseColor().R, 1.0f);
    EXPECT_FLOAT_EQ(def->GetDiffuseColor().G, 1.0f);
    EXPECT_FLOAT_EQ(def->GetDiffuseColor().B, 1.0f);
    EXPECT_FLOAT_EQ(def->GetRoughness(), 0.0f);
    EXPECT_FLOAT_EQ(def->GetMetallic(), 0.0f);
    EXPECT_FLOAT_EQ(def->GetOpacity(), 1.0f);
    EXPECT_TRUE(def->GetUseSpecularWorkflow());
}

// ChVisualShape::GetColor() is the accessor a backend is expected to consult for a shape with an
// empty material list. It must agree with Default(); if it did not, "use GetColor()" and "use
// Default()" would be two different policies and backends would split between them.
TEST(DefaultVisualMaterial, empty_material_list_reports_the_default_color) {
    ChVisualShapeBox shape(1.0, 1.0, 1.0);
    ASSERT_EQ(shape.GetNumMaterials(), 0u);

    const ChColor from_shape = shape.GetColor();
    const ChColor from_default = ChVisualMaterial::Default()->GetDiffuseColor();
    EXPECT_FLOAT_EQ(from_shape.R, from_default.R);
    EXPECT_FLOAT_EQ(from_shape.G, from_default.G);
    EXPECT_FLOAT_EQ(from_shape.B, from_default.B);
}

// -----------------------------------------------------------------------------
// Every member is initialized.
// -----------------------------------------------------------------------------

// The Hapke members had no initializer, in the constructor or in the class. Nothing gated reading
// them: ChOptixPipeline::GetMaterial copies all seven into its device struct for every material it
// converts, without testing bsdf_type first, so every render was copying indeterminate host memory
// to the GPU. It could not change a pixel, because the shader only reads those values when the
// user selected the Hapke BSDF and such a user has called SetHapkeParameters. Latent, but still
// undefined behaviour, and the fix routes one more caller through that read.
//
// Note what this test can and cannot do: reading an uninitialized float is undefined, so a build
// where it is broken may legitimately produce anything, including these values by luck. The test
// is a tripwire against the initializers being dropped again, not proof of definedness.
TEST(DefaultVisualMaterial, hapke_members_are_initialized) {
    ChVisualMaterial mat;

    EXPECT_FALSE(mat.GetUseHapke());
    EXPECT_FLOAT_EQ(mat.GetHapkeW(), 0.0f);
    EXPECT_FLOAT_EQ(mat.GetHapkeB(), 0.0f);
    EXPECT_FLOAT_EQ(mat.GetHapkeC(), 0.0f);
    EXPECT_FLOAT_EQ(mat.GetHapkeBs0(), 0.0f);
    EXPECT_FLOAT_EQ(mat.GetHapkeHs(), 0.0f);
    EXPECT_FLOAT_EQ(mat.GetHapkePhi(), 0.0f);
    EXPECT_FLOAT_EQ(mat.GetHapkeRoughness(), 0.0f);
}

// Two independently constructed materials must agree on every field a backend reads. If any member
// were left uninitialized, two stack allocations would eventually disagree, and this catches that
// without depending on which particular garbage a given run produces.
TEST(DefaultVisualMaterial, two_fresh_materials_agree_on_every_read_field) {
    ChVisualMaterial a;
    ChVisualMaterial b;

    EXPECT_FLOAT_EQ(a.GetDiffuseColor().R, b.GetDiffuseColor().R);
    EXPECT_FLOAT_EQ(a.GetSpecularColor().R, b.GetSpecularColor().R);
    EXPECT_FLOAT_EQ(a.GetEmissiveColor().R, b.GetEmissiveColor().R);
    EXPECT_FLOAT_EQ(a.GetFresnelExp(), b.GetFresnelExp());
    EXPECT_FLOAT_EQ(a.GetFresnelMin(), b.GetFresnelMin());
    EXPECT_FLOAT_EQ(a.GetFresnelMax(), b.GetFresnelMax());
    EXPECT_FLOAT_EQ(a.GetOpacity(), b.GetOpacity());
    EXPECT_FLOAT_EQ(a.GetRoughness(), b.GetRoughness());
    EXPECT_FLOAT_EQ(a.GetMetallic(), b.GetMetallic());
    EXPECT_FLOAT_EQ(a.GetAnisotropy(), b.GetAnisotropy());
    EXPECT_EQ(a.GetUseSpecularWorkflow(), b.GetUseSpecularWorkflow());
    EXPECT_EQ(a.GetBSDF(), b.GetBSDF());
    EXPECT_FLOAT_EQ(a.GetEmissivePower(), b.GetEmissivePower());
    EXPECT_EQ(a.GetUseHapke(), b.GetUseHapke());
    EXPECT_FLOAT_EQ(a.GetHapkeW(), b.GetHapkeW());
    EXPECT_FLOAT_EQ(a.GetHapkeB(), b.GetHapkeB());
    EXPECT_FLOAT_EQ(a.GetHapkeC(), b.GetHapkeC());
    EXPECT_FLOAT_EQ(a.GetHapkeBs0(), b.GetHapkeBs0());
    EXPECT_FLOAT_EQ(a.GetHapkeHs(), b.GetHapkeHs());
    EXPECT_FLOAT_EQ(a.GetHapkePhi(), b.GetHapkePhi());
    EXPECT_FLOAT_EQ(a.GetHapkeRoughness(), b.GetHapkeRoughness());
}

// SetHapkeParameters sets the seven floats and nothing else. Recorded so that defaulting
// use_hapke to false cannot be mistaken for a behaviour change: a Hapke user must already call
// SetBSDF separately, since neither this setter nor the constructor touches bsdf_type.
TEST(DefaultVisualMaterial, set_hapke_parameters_does_not_switch_the_bsdf) {
    ChVisualMaterial mat;
    mat.SetHapkeParameters(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f);

    EXPECT_FLOAT_EQ(mat.GetHapkeW(), 0.1f);
    EXPECT_FLOAT_EQ(mat.GetHapkeRoughness(), 0.7f);
    EXPECT_EQ(mat.GetBSDF(), BSDFType::PRINCIPLED);
}

// -----------------------------------------------------------------------------
// The singleton.
// -----------------------------------------------------------------------------

// Default() is one shared object, and the pipeline caches a snapshot of it rather than holding a
// reference, so it matters that it is stable.
TEST(DefaultVisualMaterial, default_is_a_stable_singleton) {
    EXPECT_EQ(ChVisualMaterial::Default(), ChVisualMaterial::Default());
}

// SetColor on a shape must never write through to the shared default. ChVisualShape guards this
// with copy-on-write; if that guard were lost, colouring one shape would recolour every
// material-less shape in every scene, including through the OptiX default path this change adds.
TEST(DefaultVisualMaterial, coloring_a_shape_does_not_mutate_the_singleton) {
    const ChColor before = ChVisualMaterial::Default()->GetDiffuseColor();

    ChVisualShapeBox shape(1.0, 1.0, 1.0);
    shape.SetColor(ChColor(0.123f, 0.456f, 0.789f));

    const ChColor after = ChVisualMaterial::Default()->GetDiffuseColor();
    EXPECT_FLOAT_EQ(after.R, before.R);
    EXPECT_FLOAT_EQ(after.G, before.G);
    EXPECT_FLOAT_EQ(after.B, before.B);
    EXPECT_FLOAT_EQ(shape.GetColor().R, 0.123f);
}

// Concurrent FIRST access to Default() is tested by utest_SEN_default_material_firstaccess, in its
// own executable. It cannot live here: several tests in this file call Default(), gtest runs them in
// declaration order, and once any of them has run, the static is built and a concurrency test races
// over nothing. Measured with the pre-fix implementation swapped back in, the same test passed in
// this binary and failed 200 of 200 runs in its own. Keeping a copy here would only add a test that
// cannot fail.

// The default material names no texture files. This is what makes installing it into the OptiX
// material pool a non-throwing operation: the only throwing paths in that conversion are the
// texture uploads, each guarded by a non-empty filename test. If someone gives the default a
// texture later, the install could throw partway and this test is where they find out.
TEST(DefaultVisualMaterial, default_names_no_textures) {
    auto def = ChVisualMaterial::Default();

    EXPECT_TRUE(def->GetKdTexture().empty());
    EXPECT_TRUE(def->GetKsTexture().empty());
    EXPECT_TRUE(def->GetNormalMapTexture().empty());
    EXPECT_TRUE(def->GetMetallicTexture().empty());
    EXPECT_TRUE(def->GetRoughnessTexture().empty());
    EXPECT_TRUE(def->GetOpacityTexture().empty());
    EXPECT_TRUE(def->GetWeightTexture().empty());
}

// -----------------------------------------------------------------------------
// Serialization.
// -----------------------------------------------------------------------------

// The fields ArchiveOut actually writes must come back unchanged. This is the baseline the next
// test is measured against: it establishes that the round trip works at all, so a failure there
// means something about the unwritten fields, not a broken archive.
TEST(DefaultVisualMaterial, archived_fields_survive_a_round_trip) {
    // Both sides must use the same variable name: CHNVP takes the archive key from the identifier,
    // so reading into a differently named variable throws "Cannot find ...". Hence the scopes.
    std::stringstream buffer;
    {
        auto material = chrono_types::make_shared<ChVisualMaterial>();
        material->SetDiffuseColor(ChColor(0.11f, 0.22f, 0.33f));
        material->SetSpecularColor(ChColor(0.44f, 0.55f, 0.66f));
        material->SetRoughness(0.75f);
        material->SetMetallic(0.25f);
        material->SetOpacity(0.5f);
        material->SetUseSpecularWorkflow(false);
        material->SetKdTexture("some/diffuse.png");

        ChArchiveOutJSON archive_out(buffer);
        archive_out << CHNVP(material);
    }

    std::shared_ptr<ChVisualMaterial> restored;
    {
        std::shared_ptr<ChVisualMaterial> material;
        ChArchiveInJSON archive_in(buffer);
        archive_in >> CHNVP(material);
        restored = material;
    }
    ASSERT_NE(restored, nullptr);

    EXPECT_FLOAT_EQ(restored->GetDiffuseColor().R, 0.11f);
    EXPECT_FLOAT_EQ(restored->GetDiffuseColor().G, 0.22f);
    EXPECT_FLOAT_EQ(restored->GetDiffuseColor().B, 0.33f);
    EXPECT_FLOAT_EQ(restored->GetSpecularColor().R, 0.44f);
    EXPECT_FLOAT_EQ(restored->GetRoughness(), 0.75f);
    EXPECT_FLOAT_EQ(restored->GetMetallic(), 0.25f);
    EXPECT_FLOAT_EQ(restored->GetOpacity(), 0.5f);
    EXPECT_FALSE(restored->GetUseSpecularWorkflow());
    EXPECT_EQ(restored->GetKdTexture(), "some/diffuse.png");
}

// The other half of the round trip, and the half this change is responsible for.
//
// ArchiveOut writes 23 of ChVisualMaterial's 40 members. It does not write emissive_power,
// anisotropy, use_hapke, bsdf_type, class_id, instance_id, or the seven Hapke floats, so a
// deserialized material takes those from the default constructor, not from the file. That is a
// pre-existing gap in ChVisualMaterial's archive and it is deliberately NOT fixed here: adding
// fields to ArchiveOut changes the on-disk format and needs a version bump, which is a separate
// change from this one.
//
// What matters here is that the constructor is now the whole story for those members. Before the
// Hapke members were initialized, deserializing a material produced eight values read from
// uninitialized stack, and ChOptixPipeline copies all of them to the GPU. So this test asserts
// determinism, not a particular value: two independent round trips of the same archive must agree
// on every unwritten member. It stays correct if someone later adds these fields to the archive.
TEST(DefaultVisualMaterial, round_trip_leaves_the_unarchived_members_deterministic) {
    std::stringstream buffer;
    {
        auto material = chrono_types::make_shared<ChVisualMaterial>();
        material->SetHapkeParameters(0.31f, 0.32f, 0.33f, 0.34f, 0.35f, 0.36f, 0.37f);
        material->SetBSDF(BSDFType::HAPKE);
        material->SetAnisotropy(0.44f);
        material->SetEmissivePower(7.5f);
        material->SetClassID(11);
        material->SetInstanceID(22);

        ChArchiveOutJSON archive_out(buffer);
        archive_out << CHNVP(material);
    }
    const std::string archived = buffer.str();

    std::shared_ptr<ChVisualMaterial> first;
    std::shared_ptr<ChVisualMaterial> second;
    {
        std::shared_ptr<ChVisualMaterial> material;
        std::stringstream in(archived);
        ChArchiveInJSON archive_in(in);
        archive_in >> CHNVP(material);
        first = material;
    }
    {
        std::shared_ptr<ChVisualMaterial> material;
        std::stringstream in(archived);
        ChArchiveInJSON archive_in(in);
        archive_in >> CHNVP(material);
        second = material;
    }
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);

    EXPECT_EQ(first->GetUseHapke(), second->GetUseHapke());
    EXPECT_FLOAT_EQ(first->GetHapkeW(), second->GetHapkeW());
    EXPECT_FLOAT_EQ(first->GetHapkeB(), second->GetHapkeB());
    EXPECT_FLOAT_EQ(first->GetHapkeC(), second->GetHapkeC());
    EXPECT_FLOAT_EQ(first->GetHapkeBs0(), second->GetHapkeBs0());
    EXPECT_FLOAT_EQ(first->GetHapkeHs(), second->GetHapkeHs());
    EXPECT_FLOAT_EQ(first->GetHapkePhi(), second->GetHapkePhi());
    EXPECT_FLOAT_EQ(first->GetHapkeRoughness(), second->GetHapkeRoughness());
    EXPECT_EQ(first->GetBSDF(), second->GetBSDF());
    EXPECT_FLOAT_EQ(first->GetAnisotropy(), second->GetAnisotropy());
    EXPECT_FLOAT_EQ(first->GetEmissivePower(), second->GetEmissivePower());
    EXPECT_EQ(first->GetClassID(), second->GetClassID());
    EXPECT_EQ(first->GetInstanceID(), second->GetInstanceID());
}

// A shape saved with an empty material list must come back with an empty material list, and so must
// resolve to the default at render time rather than to whatever a backend keeps of its own. This is
// the shape-level half of the serialization question: it is what makes the appearance change apply
// to already-saved scenes, which is the part of this change users are most likely to notice.
//
// What it does not cover: rendering a deserialized scene through the GPU backends and comparing. That
// needs a GPU and full ChSystem serialization, and it is not done.
TEST(DefaultVisualMaterial, an_empty_material_list_survives_a_round_trip) {
    std::stringstream buffer;
    {
        auto shape = chrono_types::make_shared<ChVisualShapeBox>(1.0, 2.0, 3.0);
        ASSERT_EQ(shape->GetNumMaterials(), 0u);
        ChArchiveOutJSON archive_out(buffer);
        archive_out << CHNVP(shape);
    }

    std::shared_ptr<ChVisualShapeBox> restored;
    {
        std::shared_ptr<ChVisualShapeBox> shape;
        ChArchiveInJSON archive_in(buffer);
        archive_in >> CHNVP(shape);
        restored = shape;
    }
    ASSERT_NE(restored, nullptr);

    EXPECT_EQ(restored->GetNumMaterials(), 0u);
    const ChColor from_default = ChVisualMaterial::Default()->GetDiffuseColor();
    EXPECT_FLOAT_EQ(restored->GetColor().R, from_default.R);
    EXPECT_FLOAT_EQ(restored->GetColor().G, from_default.G);
    EXPECT_FLOAT_EQ(restored->GetColor().B, from_default.B);
}

// -----------------------------------------------------------------------------
// The two creation paths.
// -----------------------------------------------------------------------------

// The inconsistency that showed this was not a deliberate convention. ChBodyEasyBox leaves the
// material list empty; chrono::utils::AddBoxGeometry defaults its vis_material argument to
// Default() and attaches it. Under the old OptiX code those rendered 0.5 grey and white
// respectively. They must now report the same colour, since both resolve to the same default.
TEST(DefaultVisualMaterial, easy_box_and_explicit_default_agree_on_color) {
    ChSystemNSC sys;

    auto easy = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.0, 1.0, 1000, true, false);
    sys.Add(easy);
    auto easy_shape = easy->GetVisualModel()->GetShapeInstances()[0].shape;
    ASSERT_EQ(easy_shape->GetNumMaterials(), 0u);

    ChVisualShapeBox explicit_shape(1.0, 1.0, 1.0);
    explicit_shape.AddMaterial(ChVisualMaterial::Default());
    ASSERT_EQ(explicit_shape.GetNumMaterials(), 1u);

    EXPECT_FLOAT_EQ(easy_shape->GetColor().R, explicit_shape.GetColor().R);
    EXPECT_FLOAT_EQ(easy_shape->GetColor().G, explicit_shape.GetColor().G);
    EXPECT_FLOAT_EQ(easy_shape->GetColor().B, explicit_shape.GetColor().B);
}
