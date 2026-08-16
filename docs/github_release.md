# GitHub release checklist

## Included

- corridor geometry and the retained 18-site station table;
- public evidence register and source URLs;
- modular corridor, station, trajectory, radio, policy, and capacity code;
- reproducible link-quality and simulator entry points;
- frozen compact reference data and figures;
- 2D/3D dashboard and locally licensed aircraft model;
- tests, checksum verification, and GitHub Actions configuration.

## Excluded

- caches, virtual environments, temporary runs, and trial folders;
- superseded station candidates and the former BS19 scenario;
- third-party Street View screenshots and downloaded government PDFs;
- unrelated AMsim4City/TRB working trees.

## Before creating the remote repository

1. Repository name: `bay-area-uam-link-quality-simulator`; visibility: public.
2. Code is MIT licensed; original data and documentation are CC BY 4.0.
3. Run `python -m pytest -q` and `python scripts/verify_reference.py`.
4. Review `RELEASE_REVIEW.md`, then create the initial commit and remote.
