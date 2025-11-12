mod model;
mod parser;

fn main() -> anyhow::Result<()> {
    // Caminho relativo ao diretório raiz do projeto
    let path = "../instances/toy_instance/toy.txt";
    let instance = parser::parse_instance(path)?;

    println!("{}", instance);

    Ok(())
}
