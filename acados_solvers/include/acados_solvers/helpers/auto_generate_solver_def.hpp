#include "acados_solvers/acados_solver.hpp"

#define PPCAT_NX(A, B) A ## B
#define PPCAT(A, B) PPCAT_NX(A, B)
#define PREPEND_IMPORT_PREFIX(FCT_NAME) PPCAT(ACADOS_PREFIX_LOWER, PPCAT(_acados_, FCT_NAME))
#define PREPEND_IMPORT_MACRO_PREFIX(MACRO_NAME) PPCAT(ACADOS_PREFIX_UPPER, PPCAT(_, MACRO_NAME))
#define PREPEND_EXPORT_PREFIX(CLASS_NAME) PPCAT(SOLVER_EXPORT_PREFIX, CLASS_NAME)


struct PPCAT (ACADOS_PREFIX_LOWER, _solver_capsule) ;

namespace acados
{

  class PREPEND_EXPORT_PREFIX (AcadosSolver) : public AcadosSolver
  {
public:
    PREPEND_EXPORT_PREFIX(AcadosSolver)();
    ~PREPEND_EXPORT_PREFIX(AcadosSolver)();

protected:
    virtual int create_index_maps();
    virtual int internal_create_capsule();
    virtual int internal_create_with_discretization(int n_time_steps, double * new_time_steps);
    virtual int internal_reset(int reset_qp_solver_mem);
    virtual int internal_free();
    virtual int internal_free_capsule();

    virtual int internal_update_qp_solver_cond_N(int qp_solver_cond_N);
    virtual int internal_update_params(int stage, double * value, int np);
    virtual int internal_update_params_sparse(int stage, int * idx, double * p, int n_update);
    virtual int internal_solve();
    virtual void internal_print_stats() const;

    virtual ocp_nlp_in * get_nlp_in() const;
    virtual ocp_nlp_out * get_nlp_out() const;
    virtual ocp_nlp_out * get_sens_out() const;
    virtual ocp_nlp_solver * get_nlp_solver() const;
    virtual ocp_nlp_config * get_nlp_config() const;
    virtual void * get_nlp_opts() const;
    virtual ocp_nlp_dims * get_nlp_dims() const;
    virtual ocp_nlp_plan_t * get_nlp_plan() const;
    virtual unsigned int get_nlp_np() const;

private:
    PPCAT(ACADOS_PREFIX_LOWER, _solver_capsule) * _capsule = nullptr;
  };

} // namespace acados
