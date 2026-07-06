//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_LAZY_LABEL_H
#define NETWORKS2019_LAZY_LABEL_H

#include "label.h"
#include "nyr/vrp/types.h"

namespace solver
{
class LazyLabel
{
public:
	Label* parent; // label to extend.
	goc::Vertex v; // extending to vertex v.
	nyr::TimeUnit makespan; // earliest arrival time to v, for queuing purposes.
	Label* extension; // if lazyness is not used, the extension is stored here.
	
	LazyLabel();
	
	LazyLabel(Label* parent, goc::Vertex v, nyr::TimeUnit makespan);
};
} // namespace

#endif //NETWORKS2019_LAZY_LABEL_H
