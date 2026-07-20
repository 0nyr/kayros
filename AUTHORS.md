# Authors

This file records the authorship, scientific supervision, funding context, and contributor policy of KAYROS. It is the authoritative statement of who wrote the software; machine-readable citation metadata lives in [`CITATION.cff`](CITATION.cff), and the license terms in [`LICENSE`](LICENSE).

## Author and maintainer

KAYROS is written and maintained by:

- **Florian Rascoussier** (Onyr), IMT Atlantique and INSA Lyon, France. [GitHub](https://github.com/0nyr) · [ORCID 0009-0005-3253-9814](https://orcid.org/0009-0005-3253-9814) · [LinkedIn](https://www.linkedin.com/in/florian-rascoussier-onyr/) · <rascoussier.florian@gmail.com>

He is the sole author of the KAYROS codebase, with the exception of the vendored third-party component described below. Everything stays MIT-licensed.

## Vendored third-party code

The exact branch-price-and-cut component (`cpp/lera/`) is derived from the open-source solver of **Gonzalo Lera-Romero**, **Juan J. Miranda Bront** and **Francisco J. Soulignac**, published alongside *Linear edge costs and labeling algorithms: The case of the time-dependent vehicle routing problem with time windows*, Networks 76(1):24–53, 2020 ([doi:10.1002/net.21937](https://doi.org/10.1002/net.21937), [source repository](https://github.com/gleraromero/networks2020)). Provenance, the list of modifications, and the attribution history are documented in [`cpp/lera/NOTICE.md`](cpp/lera/NOTICE.md).

## AI-assisted development

KAYROS is developed with the assistance of AI coding tools (Claude, ChatGPT, and others) for code generation, documentation, and exploratory work. Claude Fable 5 in particular has been used extensively. The author remains solely responsible for all design decisions, all published claims, and all content of this repository.

## Scientific supervision

KAYROS is built as part of a [PhD in Informatics and Operations Research](https://theses.fr/s386454) at IMT Atlantique and INSA Lyon, supervised by:

- [**Romain Billot**](https://www.imt-atlantique.fr/en/person/romain-billot), director of the PhD, IMT Atlantique, France
- [**Christine Solnon**](http://perso.citi.insa-lyon.fr/csolnon/), co-director of the PhD, INSA Lyon, France
- [**Lina Fahed**](https://www.imt-atlantique.fr/en/person/lina-fahed), co-supervisor of the PhD, IMT Atlantique, France

## Funding and project context

This work is part of the ANR-MAMUT project, [ANR-22-CE22-0016](https://anr.fr/Projet-ANR-22-CE22-0016).

## Acknowledgements

The KAYROS solver is dedicated to the memory of my friend Thomas Garcia.

### The ANR-MAMUT project

KAYROS has benefited from the collaboration initiated around the [ANR-MAMUT project](https://anr.fr/Projet-ANR-22-CE22-0016) and its [MAMUT-routing benchmark catalog](https://github.com/ANR-MAMUT/MAMUT-routing), where KAYROS is the reference solver for the time-dependent problem families. The following people have contributed indirectly to the project and are acknowledged for their support (alphabetical order of last name):

- [Alexandru-Liviu Olteanu](http://www-labsticc.univ-ubs.fr/~olteanu/), Université Bretagne Sud, France
- [Adrien Pichon](https://orcid.org/0009-0005-8630-3962) ([GitHub](https://github.com/Anzury)), Université Bretagne Sud, France
- [Marc Sevaux](http://www-labsticc.univ-ubs.fr/~sevaux/), Université Bretagne Sud, France

### Individual acknowledgements

- A special mention to [**Christine Solnon**](http://perso.citi.insa-lyon.fr/csolnon/) for her guidance and for her decade-long involvement in time-dependent routing, going as far back as the [PhD of Pénélope Aguiar Melgarejo](https://hal.science/hal-01514369).
- [**Romain Fontaine**](https://github.com/romainfontaine), for his help with [Grid'5000](https://www.grid5000.fr/), where every KAYROS validation and certification campaign runs. This PhD is a multi-vehicle follow-up to [his TDTSPTW thesis](https://hal.science/tel-04697323v1) and his dynamic-programming solver, [tdtsptw-ejor23](https://github.com/romainfontaine/tdtsptw-ejor23).
- [**Gonzalo Lera-Romero**](https://github.com/gleraromero), for his open-source contributions to the time-dependent routing literature: the branch-price-and-cut solver vendored and extended in KAYROS, and the [TDVRPTW benchmark best-known solutions](https://github.com/gleraromero/networks2020/blob/master/instances/dabia_et_al_2013/solutions.json) used as a reference in KAYROS and MAMUT-routing.
- [**Leon Lan**](https://github.com/leonlan), [**Niels Wouda**](https://github.com/N-Wouda), [**Wouter Kool**](https://github.com/wouterkool) and the other contributors to [PyVRP](https://github.com/PyVRP/PyVRP), a major inspiration for the KAYROS anytime layer in three respects: the realization that a single-trajectory iterated local search is a strong, simple and scalable anytime metaheuristic for vehicle routing, the overall C++/Python architecture, and the competitive drive sparked by [this post by Niels](https://github.com/PyVRP/PyVRP/issues/867#issuecomment-3045950768). At the time I had already been on this research stream for a year, and realizing I was not alone on the problem was very motivating.
- [**Thibaut Vidal**](https://github.com/vidalt), who initiated the movement of open-source vehicle routing solvers and whose [HGS-CVRP](https://github.com/vidalt/HGS-CVRP) is a reference anytime metaheuristic for the field. I have learned a lot from his work, and I am grateful for the open-source spirit he has fostered in the community.
- [**Geoffrey De Smet**](https://github.com/ge0ffrey), CTO of [Timefold](https://timefold.ai/), for his valuable feedback on the Timefold solver and his helpful private discussions.

## Contributing

KAYROS welcomes contributions through pull requests and discussions on the [KAYROS repository](https://github.com/0nyr/kayros). This includes AI-assisted contributions, which are allowed under the same MIT license as the rest of the codebase. The contributor remains solely responsible for all design decisions, all published claims, and all content of their contributions. External contributors will be listed in this section as they arrive.

There are no external contributors yet.
