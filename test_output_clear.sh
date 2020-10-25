rm -r ./csv_files/*

for d in ./rmoo_files/*; do
  if [ -d "$d" ]; then
    rm -r $d/*.rmoo
  fi
done

for d in ./rmob_files/*; do
  if [ -d "$d" ]; then
    rm -r $d/*.rmob
  fi
done