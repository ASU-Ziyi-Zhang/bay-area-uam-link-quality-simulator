# Site search, inclusion, and antenna-height method

## 1. Purpose and reference corridor

This file defines the reproducible screening rules for the first realistic SF–SJ UAM communication scenario. The spatial reference is the Caltrain alignment from San Francisco 4th & King to San Jose Diridon stored in `04_corridor_geometry/corridor_centerline.geojson`.

The resulting sites form a candidate physical-infrastructure layer for radio and capacity experiments. They are not a carrier network plan or a complete inventory.

## 2. Spatial search and final inclusion range

Two distances must be distinguished:

- **Primary search and final inclusion band:** shortest lateral distance of **0–5 km on either side** of the corridor centerline.
- **Supplementary discovery band:** **5–10 km** on either side, used during investigation only to identify possible backups near gaps or endpoints.

The final scenario applies the 5-km inclusion rule. All 18 retained sites satisfy it; their largest lateral offset is BS18 at 4.433 km. A location in the supplementary band is not automatically included.

Lateral offset is the shortest projected distance between the site coordinate and the corridor centerline, calculated in a local metric coordinate system (EPSG:26910), not a road-driving distance.

## 3. Facility-type scope

Included facility types follow the FCC macro-site vocabulary:

- `Macro_Tower`: monopole, tree-form monopole, lattice/guyed tower, or other freestanding macro tower;
- `Macro_Building`: rooftop, facade, or building-concealed macro installation, including a clock-tower concealment;
- `Macro_Other`: a macro installation on another support, here the BS06 water-tower/communications complex.

Excluded from this layer are retail stores, ordinary utility cabinets, Wi-Fi access points, streetlight-scale small cells, coverage-map points without a physical-site record, and candidates lacking sufficient location evidence.

`Concealed` describes visual concealment, not one universal structure type. A concealed site may be a faux tree or may be located inside a building feature such as a clock tower.

## 4. Evidence and selection rules

A retained site must have:

1. a city, county, or other government record documenting a wireless facility at the location; and
2. coordinates or an address that can be placed relative to the corridor.

Street View, satellite imagery, and user visual review are supporting evidence. They are useful for recognizing the physical form, but they do not independently prove current operator tenancy, current radio operation, or UAM suitability.

One physical support is counted as one physical site even if multiple operators are documented on it. Operator evidence remains a separate attribute and does not imply that all operators can jointly serve one UAM link.

## 5. Antenna-height rule

The modeled quantity is an **effective antenna height above ground level (AGL)**. It is not height above the roof.

| Condition | Model height | Evidence status |
|---|---:|---|
| Official/site record supplies a usable height | retain that value | documented or documented proxy |
| `Macro_Building`, including concealed building macro, lacks height | 50 ft = 15.240 m AGL | class-based modeling assumption |
| `Macro_Tower` lacks height | 60 ft = 18.288 m AGL | class-based modeling assumption |
| BS06 water-tower support | 56 ft 1 in = 17.094 m AGL | official record |

In the current 18-site table, 10 heights come from official/site records and 8 are class-based imputations. BS03's 77.3-ft value is a site-record top-elevation proxy and may not equal the exact antenna radiation-center height.

The imputed values make the first 3D radio run executable; they do not remove height uncertainty. Results that depend materially on height should later be checked with low/base/high sensitivity cases.

## 6. Final sample and exclusion decision

The active layer contains BS01–BS18. Former BS19 at 2981 Lone Bluff Way, San Jose, was reviewed and documented but excluded because:

- lateral offset = 6.663 km, outside the final 5-km band;
- the San Jose endpoint already has closer retained sites; and
- adding it provides limited corridor-representation value for the first scenario.

Its removal is a scenario-selection decision, not a claim that the physical site is invalid.

## 7. Evidence boundary and update rule

The 18 sites are the complete set **retained by this documented search**, not proof that no other macro sites exist. Public permit systems are fragmented, rooftop installations are difficult to discover visually, and operators do not provide a complete current public inventory.

Future sites may be added or removed through the base-station interface. Each change must record the official source, coordinates, physical form, operator evidence, lateral offset, height value and basis, inclusion decision, and revision date. The corridor, trajectory, radio, and policy layers need not be rewritten when only the site inventory changes.
