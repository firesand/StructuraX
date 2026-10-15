Name:           structurax
Version:        1.0.0
Release:        1%{?dist}
Summary:        3D Finite Element Structural Analysis & Code-Based Member Design Suite
License:        MIT
URL:            https://github.com/firesand/StructuraX
BuildArch:      x86_64

%description
StructuraX is a native 3D finite element structural analysis and code-based
member design suite supporting spatial frames, catenary cables, membrane/plate
shells, 8-node 3D solids, nonlinear links/dampers, modal/buckling/spectrum/time-history
dynamics, AISC 360-10 & Eurocode 3 steel design, and ACI 318-14 & Eurocode 2
concrete design.

%install
mkdir -p %{buildroot}%{_bindir}
mkdir -p %{buildroot}%{_datadir}/structurax/sections
mkdir -p %{buildroot}%{_datadir}/applications
mkdir -p %{buildroot}%{_datadir}/icons/hicolor/scalable/apps
install -m 0755 %{_sourcedir}/structurax %{buildroot}%{_bindir}/structurax
install -m 0644 %{_sourcedir}/structurax.desktop %{buildroot}%{_datadir}/applications/structurax.desktop
install -m 0644 %{_sourcedir}/structurax.svg %{buildroot}%{_datadir}/icons/hicolor/scalable/apps/structurax.svg
install -m 0644 %{_sourcedir}/sections/*.pro %{buildroot}%{_datadir}/structurax/sections/ || true

%files
%{_bindir}/structurax
%{_datadir}/structurax/sections/*
%{_datadir}/applications/structurax.desktop
%{_datadir}/icons/hicolor/scalable/apps/structurax.svg

