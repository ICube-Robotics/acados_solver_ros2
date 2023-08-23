namespace acados
{

PREPEND_EXPORT_PREFIX(AcadosSolver) ::PREPEND_EXPORT_PREFIX(AcadosSolver)() {
  // Fill fixed problem dimensions
  _dims.nx = PREPEND_IMPORT_MACRO_PREFIX(NX);
  _dims.nz = PREPEND_IMPORT_MACRO_PREFIX(NZ);
  _dims.np = PREPEND_IMPORT_MACRO_PREFIX(NP);
  _dims.nu = PREPEND_IMPORT_MACRO_PREFIX(NU);

  _dims.nbx = PREPEND_IMPORT_MACRO_PREFIX(NBX);
  _dims.nbx0 = PREPEND_IMPORT_MACRO_PREFIX(NBX0);
  _dims.nbxN = PREPEND_IMPORT_MACRO_PREFIX(NBXN);
  _dims.nbu = PREPEND_IMPORT_MACRO_PREFIX(NBU);

  _dims.ng = PREPEND_IMPORT_MACRO_PREFIX(NG);
  _dims.ngN = PREPEND_IMPORT_MACRO_PREFIX(NGN);

  _dims.nh = PREPEND_IMPORT_MACRO_PREFIX(NH);
  _dims.nhN = PREPEND_IMPORT_MACRO_PREFIX(NHN);
  _dims.nphi = PREPEND_IMPORT_MACRO_PREFIX(NPHI);
  _dims.nphiN = PREPEND_IMPORT_MACRO_PREFIX(NPHIN);

  _dims.ns = PREPEND_IMPORT_MACRO_PREFIX(NS);
  _dims.nsN = PREPEND_IMPORT_MACRO_PREFIX(NSN);
  _dims.nsbx = PREPEND_IMPORT_MACRO_PREFIX(NSBX);
  _dims.nsbxN = PREPEND_IMPORT_MACRO_PREFIX(NSBXN);
  _dims.nsbu = PREPEND_IMPORT_MACRO_PREFIX(NSBU);
  _dims.nsh = PREPEND_IMPORT_MACRO_PREFIX(NSH);
  _dims.nshN = PREPEND_IMPORT_MACRO_PREFIX(NSHN);
  _dims.nsg = PREPEND_IMPORT_MACRO_PREFIX(NSG);
  _dims.nsgN = PREPEND_IMPORT_MACRO_PREFIX(NSGN);
  _dims.nsphi = PREPEND_IMPORT_MACRO_PREFIX(NSPHI);
  _dims.nsphiN = PREPEND_IMPORT_MACRO_PREFIX(NSPHIN);

  _dims.ny = PREPEND_IMPORT_MACRO_PREFIX(NY);
  _dims.ny0 = PREPEND_IMPORT_MACRO_PREFIX(NY0);
  _dims.nyN = PREPEND_IMPORT_MACRO_PREFIX(NYN);

  _dims.nr = PREPEND_IMPORT_MACRO_PREFIX(NR);
}

PREPEND_EXPORT_PREFIX(AcadosSolver) ::~PREPEND_EXPORT_PREFIX(AcadosSolver)() {
  free_memory();
}


int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_create_capsule() {
  _capsule = PREPEND_IMPORT_PREFIX(create_capsule)();
  return (_capsule) ? 0 : -1;
}

int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_create_with_discretization(
  int n_time_steps,
  double * new_time_steps)
{
  return PREPEND_IMPORT_PREFIX(create_with_discretization)(_capsule, n_time_steps, new_time_steps);
}
int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_reset(int reset_qp_solver_mem) {
  return PREPEND_IMPORT_PREFIX(reset)(_capsule, reset_qp_solver_mem);
}
int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_free() {
  return PREPEND_IMPORT_PREFIX(free)(_capsule);
}
int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_free_capsule() {
  return PREPEND_IMPORT_PREFIX(free_capsule)(_capsule);
}


int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_update_qp_solver_cond_N(int qp_solver_cond_N) {
  return PREPEND_IMPORT_PREFIX(update_qp_solver_cond_N)(_capsule, qp_solver_cond_N);
}
int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_update_params(
  int stage, double * value,
  int np) {
  return PREPEND_IMPORT_PREFIX(update_params)(_capsule, stage, value, np);
}
int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_update_params_sparse(
  int stage, int * idx,
  double * p, int n_update) {
  return PREPEND_IMPORT_PREFIX(update_params_sparse)(_capsule, stage, idx, p, n_update);
}
int PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_solve() {
  return PREPEND_IMPORT_PREFIX(solve)(_capsule);
}
void PREPEND_EXPORT_PREFIX(AcadosSolver) ::internal_print_stats() const
{
  return PREPEND_IMPORT_PREFIX(print_stats)(_capsule);
}

ocp_nlp_in * PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_nlp_in() const
{
  return PREPEND_IMPORT_PREFIX(get_nlp_in)(_capsule);
}
ocp_nlp_out * PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_nlp_out() const
{
  return PREPEND_IMPORT_PREFIX(get_nlp_out)(_capsule);
}
ocp_nlp_out * PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_sens_out() const
{
  return PREPEND_IMPORT_PREFIX(get_sens_out)(_capsule);
}
ocp_nlp_solver * PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_nlp_solver() const
{
  return PREPEND_IMPORT_PREFIX(get_nlp_solver)(_capsule);
}
ocp_nlp_config * PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_nlp_config() const
{
  return PREPEND_IMPORT_PREFIX(get_nlp_config)(_capsule);
}
void * PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_nlp_opts() const
{
  return PREPEND_IMPORT_PREFIX(get_nlp_opts)(_capsule);
}
ocp_nlp_dims * PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_nlp_dims() const
{
  return PREPEND_IMPORT_PREFIX(get_nlp_dims)(_capsule);
}
ocp_nlp_plan_t * PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_nlp_plan() const
{
  return PREPEND_IMPORT_PREFIX(get_nlp_plan)(_capsule);
}
unsigned int PREPEND_EXPORT_PREFIX(AcadosSolver) ::get_nlp_np() const
{
  return _capsule->nlp_np;
}

} // namespace acados
