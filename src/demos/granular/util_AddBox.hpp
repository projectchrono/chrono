// Adds a containing box with interior of dimensions dim_x X dim_y X dim_z and
// the interior volume centered at pos
void AddBox(ChSystemSMC& m_sys,
            ChVector<> pos,   // Centroid of box
            ChVector<> dims,  // full dimensions of interior
            double thick,     // thickness of box
            double box_mass,
            std::shared_ptr<ChMaterialSurfaceSMC> box_mat) {
    hx = dims.x() / 2;
    hy = dims.y() / 2;
    hz = dims.z() / 2;
    h_thick = thick / 2;

    auto box = std::shared_ptr<ChBody>(m_sys.NewBody());
    box->SetPos(pos);
    box->SetMass(box_mass);
    box->SetMaterialSurface(box_mat);
    box->SetBodyFixed(true);
    box->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box.get(), ChVector<>(hx, hy, hthick), ChVector<>(0, 0, -(hz + hthick)));  // Bottom
    utils::AddBoxGeometry(box.get(), ChVector<>(hthick, hy, hz), ChVector<>(-(hx + hthick), 0, 0));  // Low X
    utils::AddBoxGeometry(box.get(), ChVector<>(hthick, hy, hz), ChVector<>(hx + hthick, 0, 0));     // High X
    utils::AddBoxGeometry(box.get(), ChVector<>(hx, hthick, hz), ChVector<>(0, -(hy + hthick), 0));  // Low Y
    utils::AddBoxGeometry(box.get(), ChVector<>(hx, hthick, hz), ChVector<>(0, hy + hthick, 0));     // High Y
    box->GetCollisionModel()->BuildModel();
    box->SetCollide(true);
    m_sys.AddBody(box);
}